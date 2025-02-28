//-----------------------------------------------------------------------------
// Project       : fpga-hash-table
//-----------------------------------------------------------------------------
// Author        : Ivan Shevchuk (github/johan92)
//-----------------------------------------------------------------------------
// Delete algo:
// 
//   if( no valid head_ptr )
//     DELETE_NOT_SUCCESS_NO_ENTRY
//   else
//     if( key matched )
//       begin
//         
//         clear data in addr
//         put addr to empty list 
//   
//         if( it's first data in chain ) 
//           begin
//             // update head ptr in head_table 
//             if( next_ptr is NULL )
//               head_ptr = NULL
//             else
//               head_ptr = next_ptr
//           end
//        else
//          if( it's last data in chain )
//            begin
//              set in previous chain addr next_ptr is NULL
//            end
//          else
//            // we got data in the middle of chain
//            begin
//              set in previous chain addr next_ptr is ptr of next data
//            end
//   
//         DELETE_SUCCESS
//       end
//     else
//       begin
//         DELETE_NOT_SUCCESS_NO_ENTRY
//       end


import hash_table::*;

module data_table_delete #(
  parameter RAM_LATENCY = 2,

  parameter A_WIDTH     = TABLE_ADDR_WIDTH
) ( 
  input                       clk_i,
  input                       rst_i,
  
  input  ht_pdata_t           task_i,
  input                       task_valid_i,
  output                      task_ready_o,
  
  // to data RAM
  input  ram_data_t           rd_data_i,
  output logic [A_WIDTH-1:0]  rd_addr_o,
  output logic                rd_en_o,

  output logic [A_WIDTH-1:0]  wr_addr_o,
  output ram_data_t           wr_data_o,
  output logic                wr_en_o,
  
  // to empty pointer storage
  output  [A_WIDTH-1:0]       add_empty_ptr_o,
  output                      add_empty_ptr_en_o,

  head_table_if.master        head_table_if,

  // output interface with search result
  output ht_result_t          result_o,
  output logic                result_valid_o,
  input                       result_ready_i
);

enum int unsigned {
  IDLE_S,

  NO_VALID_HEAD_PTR_S,

  READ_HEAD_S,
  GO_ON_CHAIN_S,

  IN_TAIL_WITHOUT_MATCH_S,

  KEY_MATCH_IN_HEAD_S,
  KEY_MATCH_IN_MIDDLE_S,
  KEY_MATCH_IN_TAIL_S,

  CLEAR_RAM_AND_PTR_S,

  UPDATE_VAL_WR,
  UPDATE_VAL_RD,
  RESTART_AGING

} state, next_state, state_d1;

ht_pdata_t              task_locked;
logic                   key_match;
logic                   got_tail;
logic [A_WIDTH-1:0]     rd_addr;
ram_data_t              prev_rd_data;
ram_data_t              prev_prev_rd_data;
logic [A_WIDTH-1:0]     prev_rd_addr;

logic                   rd_data_val;
logic                   rd_data_val_d1;
logic                   state_first_tick;

logic                   key_zero;
logic [3:0]            aging_counter = 0;
logic [7:0]             aging_diff = 0;
logic                   head_delete = 0;
logic                   got_tail_aux = 0;
logic                   last_updated_val = 0;
logic                   first_updated_addr;
logic                   restart = 0;
logic                   interrupt = 0;
logic                   no_head = 0;
ram_data_t              last_updated;

rd_data_val_helper #( 
  .RAM_LATENCY                          ( RAM_LATENCY  ) 
) rd_data_val_helper (
  .clk_i                                ( clk_i        ),
  .rst_i                                ( rst_i        ),

  .rd_en_i                              ( rd_en_o      ),
  .rd_data_val_o                        ( rd_data_val  )

);

always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    state <= IDLE_S;
  else
    state <= next_state;

always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    state_d1 <= IDLE_S;
  else
    state_d1 <= state;

assign state_first_tick = ( state != state_d1 );

// we need to do search, so this FSM will be similar 
// with search FSM

always_comb
  begin
    next_state = state;

    case( state )
      IDLE_S:
        begin
          if( task_valid_i && task_ready_o )
            begin
              if( task_i.head_ptr_val == 1'b0 )
                next_state = NO_VALID_HEAD_PTR_S;
              else
                begin
                  //last_updated = 0;
                  head_delete = 0;
                  //restart = 0;
                  if(!interrupt)
                    next_state = READ_HEAD_S;
                  else
                    next_state = GO_ON_CHAIN_S;  
                end
            end
        end

      READ_HEAD_S, GO_ON_CHAIN_S:
        begin
          interrupt = 1;
          if( rd_data_val )
            begin
              if( key_match )
                begin
                  if( state == READ_HEAD_S )
                    next_state = KEY_MATCH_IN_HEAD_S;
                  else
                    if( got_tail )
                      next_state = KEY_MATCH_IN_TAIL_S;
                    else
                      next_state = KEY_MATCH_IN_MIDDLE_S;
                end                      
                else 
                  begin
                    if( got_tail)
                      next_state = IN_TAIL_WITHOUT_MATCH_S;
                    else
                      if( task_valid_i && task_ready_o )
                        next_state = GO_ON_CHAIN_S;
                      else
                        next_state = IDLE_S;    
                  end

                if(key_zero)
                begin
                  if( state == READ_HEAD_S || state == GO_ON_CHAIN_S)
                  begin
                    
                    aging_diff = aging_counter - rd_data_i.value;
                    if(aging_diff > aging_counter)
                      aging_diff = 8'hFF - aging_diff + 1'b1;

                    if(aging_diff < 2)
                    //if(rd_data_i.value  < 2) //condition aging_counter
                      begin
                        next_state = UPDATE_VAL_WR; //GO_ON_CHAIN_S; 
                        head_delete = 0;
                        //last_updated_val = 0;
                      end
                    else
                      begin
                        if(state == READ_HEAD_S)
                            next_state = KEY_MATCH_IN_HEAD_S;
                          else 
                          begin
                            if(got_tail) 
                              next_state = KEY_MATCH_IN_TAIL_S; 
                            else
                              next_state = KEY_MATCH_IN_MIDDLE_S;
                          end
                      end
                      got_tail_aux = got_tail;
                  end 
                  //stop cycle when table is empty
                  if(rd_data_i.value == '0 && rd_data_i.key == '0) 
                  begin
                    next_state = READ_HEAD_S; //IDLE_S;
                    head_delete = 1;
                    no_head = 1;
                  end   
                end
            end
        end

      UPDATE_VAL_WR:
        begin
          next_state = UPDATE_VAL_RD;
          interrupt = 0;
          //last_updated_val = 1;
        end

      RESTART_AGING:
        begin
          restart = 1;
          interrupt = 0;
          //if(first_updated_addr == '0)
            next_state = READ_HEAD_S;
          //else
            //next_state = GO_ON_CHAIN_S;  
        end    
      
      KEY_MATCH_IN_HEAD_S, KEY_MATCH_IN_MIDDLE_S, KEY_MATCH_IN_TAIL_S:
        begin
          next_state = CLEAR_RAM_AND_PTR_S;

          if(state == KEY_MATCH_IN_HEAD_S)
            head_delete = 1;
          else
            head_delete = 0;   
        end

      CLEAR_RAM_AND_PTR_S, NO_VALID_HEAD_PTR_S, IN_TAIL_WITHOUT_MATCH_S, UPDATE_VAL_RD:
        begin
          interrupt = 0;
          //no_head = 0;
          if(!key_zero)
            got_tail_aux = got_tail;

          // waiting for accepting report 
          if( (result_valid_o && result_ready_i) || key_zero)
              if(got_tail_aux && got_tail) begin
                if(key_zero)
                  next_state = RESTART_AGING; //IDLE_S; //READ_HEAD_S;
                else  
                  next_state = IDLE_S;

                head_delete = 0;
              end
              else
                begin
                    rd_addr = prev_rd_addr; 

                  if(head_delete)
                    next_state = READ_HEAD_S;
                  else
                    next_state = GO_ON_CHAIN_S;
                end
          else
            next_state = IDLE_S;
        end

      default:
        begin
          next_state = IDLE_S;
        end
    endcase
  end

always_ff @(posedge clk_i)
  if( rst_i )
    //aging_counter = 32'h000000000;
    aging_counter = 4'h0;
  else
    aging_counter = aging_counter + 1'b1;  

always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    task_locked <= '0;
  else
    if( task_ready_o && task_valid_i )
      task_locked <= task_i;

assign key_match = ( task_locked.cmd.key == rd_data_i.key && key_zero == 0 );
assign key_zero  = ( task_i.cmd.key == 32'h00000000 );
assign got_tail  = ( rd_data_i.next_ptr_val == 1'b0 );

always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    begin
      rd_addr      <= '0;
      prev_rd_addr <= '0;
    end
  else
  if(next_state == GO_ON_CHAIN_S || no_head == 1)
    begin
      if(state == UPDATE_VAL_RD)
        rd_addr <= prev_rd_data.next_ptr;
      //else
      if(state == CLEAR_RAM_AND_PTR_S)
        rd_addr <= prev_rd_data.next_ptr;
      if(state == READ_HEAD_S)
        rd_addr <= prev_rd_addr;  
    end


always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    begin
      rd_addr      <= '0;
      prev_rd_addr <= '0;
    end
  else
    if( ( state == IDLE_S || head_delete) && ( next_state == READ_HEAD_S && !no_head) )
      begin
        if(!head_delete)
        begin
          rd_addr      <= task_i.head_ptr;
          prev_rd_addr <= rd_addr;
        end
        else
        begin
          rd_addr <= prev_rd_data.next_ptr;
          prev_rd_addr <= rd_addr;
        end
      end
    else
      if( (rd_data_val) && ( next_state == GO_ON_CHAIN_S /*&& state != READ_HEAD_S*/) ) 
        begin
          if(!head_delete) 
            begin
              rd_addr      <= rd_data_i.next_ptr;
              prev_rd_addr <= rd_addr;
            end
          else
            begin
              rd_addr <= prev_rd_data.next_ptr;
              prev_rd_addr <= rd_addr;
            end
        end

always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    rd_data_val_d1 <= 1'b0;
  else
    rd_data_val_d1 <= rd_data_val;

always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    begin
      prev_rd_data <= '0;
      prev_prev_rd_data <= '0;
    end
  else
    if( rd_data_val && next_state != IDLE_S)
      begin
        prev_rd_data      <= rd_data_i;
        prev_prev_rd_data <= prev_rd_data;
      end


assign task_ready_o = ( state == IDLE_S );

assign rd_en_o      = ( state_first_tick || rd_data_val_d1 ) && ( ( state == READ_HEAD_S   ) ||
                                                                  //( state == UPDATE_VAL_RD ) ||
                                                                  ( state == GO_ON_CHAIN_S ) );   

assign rd_addr_o    = rd_addr; 

assign wr_en_o      = state_first_tick && ( ( state == KEY_MATCH_IN_MIDDLE_S  ) ||
                                            ( state == KEY_MATCH_IN_TAIL_S    ) ||
                                            ( state == UPDATE_VAL_WR          ) ||
                                            ( state == CLEAR_RAM_AND_PTR_S    ) );

ram_data_t rd_data_locked;

always_ff @( posedge clk_i )
  if( rd_data_val )
    rd_data_locked <= rd_data_i;

always_comb
  begin
    wr_data_o = prev_prev_rd_data;
    wr_addr_o = 'x;

    case( state )

      RESTART_AGING:
        begin
          rd_addr = '0; //first_updated_addr;
          wr_addr_o = first_updated_addr;
          prev_prev_rd_data = 0;
          prev_prev_rd_data.next_ptr = first_updated_addr;
          prev_rd_data = 0;
          prev_rd_addr = first_updated_addr;
          last_updated_val = 0;
          //last_updated = first_updated_addr
          head_delete = 0;
        end

      UPDATE_VAL_WR:
        begin
          if(last_updated_val == 0)
            first_updated_addr = prev_rd_addr;

          wr_data_o = rd_data_locked;
          wr_data_o.value = rd_data_locked.value + 1'b1; 
          //wr_addr_o = prev_prev_rd_data.next_ptr; //prev_rd_addr;
          wr_addr_o = rd_addr; 
          rd_data_locked = wr_data_o;
          prev_rd_addr = prev_prev_rd_data.next_ptr;
          //rd_addr = prev_rd_data.next_ptr;
        end

      UPDATE_VAL_RD:
        begin
          //rd_addr = prev_rd_data.next_ptr;
          //wr_addr_o = prev_prev_rd_data.next_ptr;
          //wr_data_o = rd_data_locked;
          prev_rd_data = rd_data_locked;
          last_updated = rd_data_locked;
          last_updated_val = 1;         
        end  

      CLEAR_RAM_AND_PTR_S:
        begin
          wr_data_o = '0; 

          if(!key_zero)
            wr_addr_o = rd_addr;
          else
            if(!no_head)
              wr_addr_o = prev_prev_rd_data.next_ptr;
            else
            begin
              wr_addr_o = prev_rd_addr;
              no_head = 0;
            end    
        end

      KEY_MATCH_IN_MIDDLE_S:
        begin
          if(last_updated_val)
            begin
              wr_data_o = last_updated;
              wr_data_o.next_ptr     = rd_data_locked.next_ptr;
              wr_data_o.next_ptr_val = rd_data_locked.next_ptr_val;
            end
          else
            begin
              wr_data_o.next_ptr     = rd_data_locked.next_ptr;
              wr_data_o.next_ptr_val = rd_data_locked.next_ptr_val;            
            end

          wr_addr_o              = prev_rd_addr;
        end

      
      KEY_MATCH_IN_TAIL_S:
        begin
          if(last_updated_val)
            begin
              wr_data_o = last_updated;
              wr_data_o.next_ptr     = '0;
              wr_data_o.next_ptr_val = 1'b0;
            end
          else
            begin
              wr_data_o.next_ptr     = '0;
              wr_data_o.next_ptr_val = 1'b0;
            end

          wr_addr_o              = prev_rd_addr;
        end

      default:
        begin
          // do nothing
          wr_data_o = prev_prev_rd_data;
          wr_addr_o = 'x;
        end
    endcase
  end

// ******* Head Ptr table magic *******
assign head_table_if.wr_addr          = task_locked.bucket; 
assign head_table_if.wr_data_ptr      = rd_data_locked.next_ptr;
assign head_table_if.wr_data_ptr_val  = rd_data_locked.next_ptr_val;
assign head_table_if.wr_en            = state_first_tick && ( state == KEY_MATCH_IN_HEAD_S );

// ******* Empty ptr storage ******

assign add_empty_ptr_o     = rd_addr;
assign add_empty_ptr_en_o  = state_first_tick && ( state == CLEAR_RAM_AND_PTR_S );

// ******* Result calculation *******
assign result_o.cmd         = task_locked.cmd;
assign result_o.bucket      = task_locked.bucket;
assign result_o.found_value = '0;
assign result_o.rescode     = ( ( state == NO_VALID_HEAD_PTR_S     ) ||
                                ( state == IN_TAIL_WITHOUT_MATCH_S ) ) ? ( DELETE_NOT_SUCCESS_NO_ENTRY ):
                                                                         ( DELETE_SUCCESS              );

ht_chain_state_t chain_state;

always_ff @( posedge clk_i or posedge rst_i )
  if( rst_i )
    chain_state <= NO_CHAIN;
  else
    if( state != next_state )
      begin
        case( next_state )
          NO_VALID_HEAD_PTR_S     : chain_state <= NO_CHAIN;
          IN_TAIL_WITHOUT_MATCH_S : chain_state <= IN_TAIL_NO_MATCH;
          KEY_MATCH_IN_HEAD_S     : chain_state <= IN_HEAD;
          KEY_MATCH_IN_MIDDLE_S   : chain_state <= IN_MIDDLE;
          KEY_MATCH_IN_TAIL_S     : chain_state <= IN_TAIL;
          // no default: just keep old value
        endcase
      end

assign result_o.chain_state = chain_state; 

assign result_valid_o = (( state == CLEAR_RAM_AND_PTR_S      ) ||
                        ( state == NO_VALID_HEAD_PTR_S      ) ||
                        ( state == IN_TAIL_WITHOUT_MATCH_S  )) && (key_zero != 1);


// synthesis translate_off
`include "C:/Users/user/Desktop/fpga-hash-table/tb/ht_dbg.vh"

function void print_state_transition( );
  string msg;

  if( next_state != state )
    begin
      $sformat( msg, "%s -> %s", state, next_state );
      print( msg );
    end
endfunction

logic [A_WIDTH-1:0] rd_addr_latched;

always_latch
  begin
    if( rd_en_o )
      rd_addr_latched <= rd_addr_o;
  end

always_ff @( posedge clk_i )
  begin
    if( task_valid_i && task_ready_o )
      print_new_task( task_i );
    
    if( rd_data_val )
      print_ram_data( "RD", rd_addr_latched, rd_data_i );

    if( wr_en_o )
      print_ram_data( "WR", wr_addr_o, wr_data_o );
    
    if( result_valid_o && result_ready_i )
      print_result( "RES", result_o );

    print_state_transition( );
  end

// synthesis translate_on
                      
endmodule
