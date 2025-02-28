///////////////////////////////////////////////////////////////////////////////
// $Id: ethernet_parser_64bit.v 2201 2007-08-21 06:52:51Z jnaous $
//
// Module: ethernet_parser_64bit.v
// Project: NF2.1
// Description: parses the Ethernet header for a 64 bit datapath
//
///////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

`include "onet_defines.v"

import hash_table::*;

module ethernet_parser_64bit
    #(parameter DATA_WIDTH = 64,
      parameter CTRL_WIDTH=DATA_WIDTH/8,
      parameter NUM_IQ_BITS = 3,
      parameter INPUT_ARBITER_STAGE_NUM = 2
      )
   (// --- Interface to the previous stage
    input  [DATA_WIDTH-1:0]            in_data,
    input  [CTRL_WIDTH-1:0]            in_ctrl,
    input                              in_wr,

    // --- Interface to output_port_lookup
    output reg [47:0]                  dst_mac,
    output reg [47:0]                  src_mac,
    //output reg [15:0]                  ethertype,
    output reg                         eth_done,
    output reg [NUM_IQ_BITS-1:0]       src_port,
    output reg [DATA_WIDTH-1:0]        output_A_FINAL,
    output reg [DATA_WIDTH-1:0]        output_B_FINAL,
    //output reg [15:0]                  SeqNum_wr,
    output reg [NUM_IQ_BITS-1:0]       dst_port, 
    output logic                       not_found_table,

    // --- Misc
    input                              PRP,
    input                              reset,
    input                              clk
   );
    
   

   // ------------ Internal Params --------
   parameter NUM_STATES    = 12;
   parameter READ_WORD_1   = 1;
   parameter READ_WORD_2   = 2;
   parameter READ_WORD_3   = 3;
   parameter READ_WORD_4   = 4;
   parameter PROCESS_SHIFT = 5;
   parameter SHIFT_WORDS   = 6;
   parameter READ_PRP_tailer = 7;
   parameter ADD_CRC       = 8;
   parameter COMPLETE_CRC     = 9;
   parameter CHECK_PRP     = 10;
   parameter ADD_PRP_tailer = 11;
   parameter WAIT_EOP      = 13;
   parameter WAIT_RESULT   = 14;
   parameter VLANheader1   = 16'h8100;
   parameter VLANheader2   = 16'h88a8;
   parameter ethertypeID   = 'd1536;
   parameter PRP_ID        = 16'h88fb;

   // ------------- Regs/ wires -----------

   reg [NUM_STATES-1:0]                state;
   reg [NUM_STATES-1:0]                state_next;

   reg [15:0]                          ethertype;
   reg [47:0]                          dst_mac_next;
   reg [47:0]                          src_mac_next;
   reg [15:0]                          ethertype_next;
   reg [15:0]                          TPID1_next;
   reg [15:0]                          TCI1_next;
   reg [15:0]                          TPID2_next;
   reg [15:0]                          TCI2_next;
   reg [15:0]                          TPID1;
   reg [15:0]                          TCI1;
   reg [15:0]                          TPID2;
   reg [15:0]                          TCI2;
   reg                                 eth_done_next;
   reg [NUM_IQ_BITS-1:0]               src_port_next;
   //reg [NUM_IQ_BITS-1:0]               dst_port_next;
   //reg [3:0]                           dst_port;
   reg [3:0]                           dst_port_next;
   reg [3:0]                           LAN;
   reg [3:0]                           LAN_next;
   reg [15:0]                          length;
   reg [15:0]                          length_next;
   reg [15:0]                          length_aux;
   reg                                 shift32;
   reg [2:0]                           VLANheaders;
   reg [3:0]                           IHL_next;
   reg [3:0]                           IHL_aux;
   reg [3:0]                           IHL;
   reg [15:0]                          counter = 0;
   reg [31:0]                          CRC;
   reg [31:0]                          CRC_next;

   reg[15:0]                           len_shift_w;
   reg[3:0]                            len_shift_b;
   //PRP regs to store valures
   reg[15:0]                           SeqNum_next;
   reg[15:0]                           SeqNum;
   reg[11:0]                           LSDU_next;
   reg[11:0]                           LSDU;
   reg[15:0]                           PRP_suffix_next;
   reg[15:0]                           PRP_suffix;

   //PRP internal regs. Values to add to non PRP frame
   reg[11:0]                           LSDR_wr;
   reg[3:0]                            LAN_wr;
   reg [15:0]                          SeqNum_wr;

   reg [DATA_WIDTH-1:0]                output_A;
   reg [DATA_WIDTH-1:0]                output_B;

   reg [DATA_WIDTH-1:0]                output_temp_1;
   reg [DATA_WIDTH-1:0]                output_temp_2;
   reg [DATA_WIDTH-1:0]                output_temp_3;

   reg [1:0]                           PRP_words;
   reg [1:0]                           SAN_words;
   reg [11:0]                          header_words; // number of 8b, like length
   reg [11:0]                          header_words_next;

   
   ht_cmd_if ht_cmd_in(clk);
   ht_res_if ht_res_out(clk);
    
    
  hash_table_top HT(
    .clk_i          (clk),
    .rst_i          (reset),
    .ht_cmd_in      (ht_cmd_in.slave),
    .ht_res_out     (ht_res_out.master)
);

`include "CRC32_D64.v"
   // ------------ Logic ----------------

   always @(*) begin
      dst_mac_next     = dst_mac;
      src_mac_next     = src_mac;
      ethertype_next   = ethertype;
      eth_done_next    = eth_done;
      src_port_next    = src_port;
      dst_port_next    = dst_port;
      TPID1_next       = TPID1;
      TPID2_next       = TPID2;
      TCI1_next     = TCI1;
      TCI2_next     = TCI2;
      state_next       = state;
      length_next      = length;
      IHL_next         = IHL;
      LAN_next         = LAN;
      CRC_next         = CRC;
      SeqNum_next      = SeqNum;
      LSDU_next        = LSDU;
      PRP_suffix_next  = PRP_suffix;
      header_words_next= header_words;

      case(state)
        /* read the input source header and get the first word */
        READ_WORD_1: begin
           if(in_wr && in_ctrl==`IO_QUEUE_STAGE_NUM) begin
              src_port_next = in_data[`IOQ_SRC_PORT_POS + NUM_IQ_BITS - 1 : `IOQ_SRC_PORT_POS];
           end
           else if(in_wr && in_ctrl==0) begin
              dst_mac_next          = in_data[63:16];
              src_mac_next[47:32]   = in_data[15:0];
              state_next            = READ_WORD_2;

              output_A              = in_data;
              output_B              = in_data;
              
              CRC_next = nextCRC32_D64(in_data, 32'hFFFFFFFF);

              header_words_next= header_words_next + 8;
           end
        end // case: READ_WORD_1

        READ_WORD_2: begin
           if(in_wr) begin
              src_mac_next [31:0]   = in_data[63:32];
              header_words_next= header_words_next + 4;
              if(in_data[31:16] == VLANheader2) begin     
                  TPID2_next        = in_data[31:16];
                  TCI2_next      = in_data[15:0]; 
                  state_next        = READ_WORD_3;
                  VLANheaders       = 2'b10;

                  output_A              = in_data;
                  output_B              = in_data;

                  CRC_next = nextCRC32_D64(in_data, CRC);
                  header_words_next= header_words_next + 4;
              end
              else if(in_data[31:16] == VLANheader1) begin
                           TPID1_next        = in_data[31:16];
                           TCI1_next      = in_data[15:0]; 
                           state_next        = READ_WORD_4;
                           VLANheaders       = 2'b01;

                           output_A              = in_data;
                           output_B              = in_data;

                           CRC_next = nextCRC32_D64(in_data, CRC);
                           header_words_next= header_words_next + 4;
                   end
                   else begin //ler ethertype sem campos VLAN
                        if(in_data[31:16] >= ethertypeID) begin  
                           ethertype_next         = in_data[31:16];
                           if(!PRP)
                              eth_done_next          = 1;
                           IHL_next               = in_data[11:8];
                           VLANheaders            = 2'b00;
                           state_next             = READ_WORD_4;

                           output_A              = in_data;
                           output_B              = in_data;

                           CRC_next = nextCRC32_D64(in_data, CRC);
                           header_words_next= header_words_next + 3;
                        end
                        else begin
                           VLANheaders           = 2'b00;
                           length_next           = in_data[31:16];
                           state_next            = PROCESS_SHIFT;

                           output_A              = in_data;
                           output_B              = in_data;

                           if(src_port == 0 && PRP == 1) begin
                              output_A[31:16]       = in_data[31:16] + 6;
                              output_B[31:16]       = in_data[31:16] + 6;
                           end
                           
                           CRC_next = nextCRC32_D64(output_A, CRC); 
                           header_words_next= header_words_next + 2;
                        end
                   end          
           end
        end

        READ_WORD_3: begin
            if(in_wr) begin
                  TPID1_next                = in_data[63:48]; 
                  TCI1_next              = in_data[47:32]; 
                  header_words_next= header_words_next + 4;
                  if(in_data[31:16] >= ethertypeID) begin  
                     ethertype_next         = in_data[31:16];
                     if(!PRP)
                        eth_done_next          = 1;
                     IHL_next               = in_data[11:8];
                     state_next             = READ_WORD_4;

                     output_A              = in_data;
                     output_B              = in_data;

                     CRC_next = nextCRC32_D64(in_data, CRC);
                     header_words_next= header_words_next + 3;
                  end
                  else begin
                     length_next            = in_data[31:16];
                     state_next             = PROCESS_SHIFT;

                     output_A              = in_data;
                     output_B              = in_data;

                     if(src_port == 0 && PRP == 1) begin
                        output_A[31:16]       = in_data[31:16] + 6;
                        output_B[31:16]       = in_data[31:16] + 6;
                     end

                     CRC_next = nextCRC32_D64(output_A, CRC);
                     header_words_next= header_words_next + 4;
                  end
            end    
        end

        READ_WORD_4: begin 
            if(in_wr) begin
               if(VLANheaders == 1) begin
                  if(in_data[63:48] >= ethertypeID) begin
                     ethertype_next        = in_data[63:48];
                     if(!PRP)
                        eth_done_next         = 1;
                     IHL_next              = in_data[43:40];
                     length_next           = in_data[31:16];
                     state_next            = PROCESS_SHIFT;

                     output_A              = in_data;
                     output_B              = in_data;

                     if(src_port == 0 && PRP == 1) begin
                        output_A[31:16]       = in_data[31:16] + 6;
                        output_B[31:16]       = in_data[31:16] + 6;
                     end

                     CRC_next = nextCRC32_D64(output_A, CRC);
                     header_words_next= header_words_next + 6;
                  end
                  else begin
                     length_next           = in_data[63:48];
                     state_next            = PROCESS_SHIFT;

                     output_A              = in_data;
                     output_B              = in_data;

                     if(src_port == 0 && PRP == 1) begin
                        output_A[63:48]       = in_data[63:48] + 6;
                        output_B[63:48]       = in_data[63:48] + 6;
                     end

                     CRC_next = nextCRC32_D64(output_A, CRC);
                     header_words_next= header_words_next + 2;
                  end   
               end
               else begin
                  length_next              = in_data[63:48];
                  state_next               = PROCESS_SHIFT; 

                  output_A                 = in_data;
                  output_B                 = in_data;

                  /*if(src_port == 0 && PRP == 1) begin
                     output_A[31:16]       = in_data[31:16] + 6;
                     output_B[31:16]       = in_data[31:16] + 6;
                  end

                  CRC_next = nextCRC32_D64(output_A, CRC);
                  header_words_next= header_words_next + 2;*/
                  if(src_port == 0 && PRP == 1) begin
                        output_A[63:48]       = in_data[63:48] + 6;
                        output_B[63:48]       = in_data[63:48] + 6;
                     end

                     CRC_next = nextCRC32_D64(output_A, CRC);
                     header_words_next= header_words_next + 2;
               end
            end   
        end

        PROCESS_SHIFT: begin //Calc number of words to shift (payload)
         output_A = in_data;
         output_B = in_data;

         if(ethertype >= ethertypeID /*ethertype == 'h0800 eth_done == 1*/)
            length_aux = length_next - 4'b0100; //descontar a word que contém o length e já pertence ao header IPv4
         else
            length_aux = length_next;

         if(src_port != 0 && PRP) begin
            length_aux = length_aux - 4'b0110;
         end

         len_shift_w = length_aux[15:3];
           
         len_shift_b = length_aux[2:0];

         if(ethertype == 'h88ba)
            counter = 2;
         else
            counter = 1; 

         state_next = SHIFT_WORDS;

         CRC_next = nextCRC32_D64(in_data, CRC);
        end

        SHIFT_WORDS: begin //shift until the end of payload
           if(counter == len_shift_w) begin
              CRC_next = nextCRC32_D64(in_data, CRC);
               if(PRP == 0) begin
                  state_next = ADD_CRC; //modificar ADD_CRC para o caso não PRP
               end
               else begin
                  if(src_port == 0) begin //non PRP msg from internal CPU
                     state_next = ADD_PRP_tailer;
                  end
                  else begin //PRP message/message from SAN
                     state_next = READ_PRP_tailer;
                  end   
               end  
           end
           else begin
               output_A              = in_data;
               output_B              = in_data;
               CRC_next = nextCRC32_D64(in_data, CRC);
           end  
        end

        ADD_PRP_tailer: begin //ADD PRP tailer if packet comes from SRC_PRT 0
           if(in_wr == 1) begin
               //SeqNum_wr = 'h0001;
               LSDR_wr = length_next - header_words_next - 4;

               if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin 
                     case(len_shift_b) 
                        0: begin
                           dst_port_next      = in_data[15:12];

                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[15:0]     = SeqNum_wr;
                           state_next         = ADD_CRC;
                           end
                     
                        1: begin
                           dst_port_next      = in_data[7:4];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[7:0]      = SeqNum_wr[15:8];
                           state_next         = ADD_CRC;
                           end
                        
                        2: begin
                           dst_port_next      = in_data[63:60];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[63:48]    = SeqNum_wr;
                           output_temp_1[47:44]    = LAN_wr; 
                           output_temp_1[43:32]    = LSDR_wr;
                           output_temp_1[31:16]    = PRP_ID;
                           output_temp_1[15:0]     = CRC_next[31:16];
                           state_next         = ADD_CRC;
                           end
                     
                        3: begin
                           dst_port_next      = in_data[55:52];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[55:40]    = SeqNum_wr;
                           output_temp_1[39:36]    = LAN_wr;
                           output_temp_1[35:24]    = LSDR_wr;
                           output_temp_1[23:8]     = PRP_ID;
                           output_temp_1[7:0]      = CRC_next[31:24];
                           state_next         = ADD_CRC;
                           end

                        4: begin
                           dst_port_next      = in_data[47:44];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[47:32]    = SeqNum_wr;
                           output_temp_1[31:28]    = LAN_wr;
                           output_temp_1[27:16]    = LSDR_wr;
                           output_temp_1[15:0]     = PRP_ID;
                           state_next         = ADD_CRC;
                           end

                        5: begin
                           dst_port_next      = in_data[39:36];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[39:24]    = SeqNum_wr;
                           output_temp_1[23:20]    = LAN_wr;
                           output_temp_1[19:8]     = LSDR_wr;
                           output_temp_1[7:0]      = PRP_ID[15:8];
                           state_next         = ADD_CRC;
                           end

                        6: begin
                           dst_port_next      = in_data[31:28];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[31:16]    = SeqNum_wr;
                           output_temp_1[15:12]    = LAN_wr;
                           output_temp_1[11:0]     = LSDR_wr;
                           state_next         = ADD_CRC;
                           end

                        7: begin
                           dst_port_next      = in_data[23:20];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[23:8]     = SeqNum_wr;
                           output_temp_1[7:4]      = LAN_wr;
                           output_temp_1[3:0]      = LSDR_wr[11:8];
                           state_next         = ADD_CRC;
                           end
                     endcase
               end
               else begin
                     case(len_shift_b) 
                        0: begin
                           dst_port_next      = in_data[47:44];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[47:32]    = SeqNum_wr;
                           output_temp_1[31:28]    = LAN_wr;
                           output_temp_1[27:16]    = LSDR_wr;
                           output_temp_1[15:0]     = PRP_ID;
                           state_next         = ADD_CRC;
                           end
                     
                        1: begin
                           dst_port_next      = in_data[39:36];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[39:24]    = SeqNum_wr;
                           output_temp_1[23:20]    = LAN_wr;
                           output_temp_1[19:8]     = LSDR_wr;
                           output_temp_1[7:0]      = PRP_ID[15:8];
                           state_next         = ADD_CRC;
                           end
                        
                        2: begin
                           dst_port_next      = in_data[31:28];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[31:16]    = SeqNum_wr;
                           output_temp_1[15:12]    = LAN_wr;
                           output_temp_1[11:0]     = LSDR_wr;
                           state_next         = ADD_CRC;
                           end

                        3: begin
                           dst_port_next      = in_data[23:20];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[23:8]     = SeqNum_wr;
                           output_temp_1[7:4]      = LAN_wr;
                           output_temp_1[3:0]      = LSDR_wr[11:8];
                           state_next         = ADD_CRC;
                           end

                        4: begin
                           dst_port_next      = in_data[15:12];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[15:0]     = SeqNum_wr;
                           state_next         = ADD_CRC;
                           end

                        5: begin
                           dst_port_next      = in_data[7:4];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[7:0]      = SeqNum_wr[15:8];
                           state_next         = ADD_CRC;
                           end

                        6: begin
                           dst_port_next      = in_data[63:60];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[63:48]    = SeqNum_wr;
                           output_temp_1[47:44]    = LAN_wr; 
                           output_temp_1[43:32]    = LSDR_wr;
                           output_temp_1[31:16]    = PRP_ID;
                           output_temp_1[15:0]     = CRC_next[31:16];
                           state_next         = ADD_CRC;
                           end

                        7: begin
                           dst_port_next      = in_data[55:52];
                           case(dst_port_next) 
                              1: LAN_wr = 4'b0001;
                              2: LAN_wr = 4'b0010;
                              3: LAN_wr = 4'b0001;
                              4: LAN_wr = 4'b0010;
                              5: LAN_wr = 4'b0001;
                              6: LAN_wr = 4'b0010;
                              7: LAN_wr = 4'b0001;
                              8: LAN_wr = 4'b0010;
                           endcase
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[55:40]    = SeqNum_wr;
                           output_temp_1[39:36]    = LAN_wr;
                           output_temp_1[35:24]    = LSDR_wr;
                           output_temp_1[23:8]     = PRP_ID;
                           output_temp_1[7:0]      = CRC_next[31:24];
                           state_next         = ADD_CRC;
                           end                    
                     endcase
               end  
           end
           dst_port = LAN_wr;
           output_A_FINAL = output_temp_1;
           output_B_FINAL = output_temp_1;
        end

        READ_PRP_tailer: begin //read PRP tailer when packet comes from LAN
            if(in_wr) begin
               if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin 
                     case(len_shift_b) 
                        0: begin
                           SeqNum_next        = in_data[15:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end
                     
                        1: begin
                           SeqNum_next[15:8]  = in_data[7:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end
                        
                        2: begin
                           SeqNum_next        = in_data[63:48];
                           LAN_next           = in_data[47:44];
                           LSDU_next          = in_data[43:32];
                           PRP_suffix_next    = in_data[31:16];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[15:0]     = CRC_next[31:16];
                           state_next         = ADD_CRC;
                           end
                     
                        3: begin
                           SeqNum_next        = in_data[55:40];
                           LAN_next           = in_data[39:36];
                           LSDU_next          = in_data[35:24];
                           PRP_suffix_next    = in_data[23:8];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[7:0]      = CRC_next[31:24];
                           state_next         = ADD_CRC;
                           end

                        4: begin
                           SeqNum_next        = in_data[47:32];
                           LAN_next           = in_data[31:28];
                           LSDU_next          = in_data[27:16];
                           PRP_suffix_next    = in_data[15:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end

                        5: begin
                           SeqNum_next        = in_data[39:24];
                           LAN_next           = in_data[23:20];
                           LSDU_next          = in_data[19:8];
                           PRP_suffix_next[15:8]   = in_data[7:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end

                        6: begin
                           SeqNum_next        = in_data[31:16];
                           LAN_next           = in_data[15:12];
                           LSDU_next          = in_data[11:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end

                        7: begin
                           SeqNum_next        = in_data[23:8];
                           LAN_next           = in_data[7:4];
                           LSDU_next[11:8]    = in_data[3:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end
                     endcase
               end
               else begin
                     case(len_shift_b) 
                        0: begin
                           SeqNum_next        = in_data[47:32];
                           LAN_next           = in_data[31:28];
                           LSDU_next          = in_data[27:16];
                           PRP_suffix_next    = in_data[15:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end
                     
                        1: begin
                           SeqNum_next        = in_data[39:24];
                           LAN_next           = in_data[23:20];
                           LSDU_next          = in_data[19:8];
                           PRP_suffix_next[15:8]    = in_data[7:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end
                        
                        2: begin
                           SeqNum_next        = in_data[31:16];
                           LAN_next           = in_data[15:12];
                           LSDU_next          = in_data[11:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end

                        3: begin
                           SeqNum_next        = in_data[23:8];
                           LAN_next           = in_data[7:4];
                           LSDU_next[11:8]    = in_data[3:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end

                        4: begin
                           SeqNum_next        = in_data[15:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end

                        5: begin
                           SeqNum_next[15:8]  = in_data[7:0];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           state_next         = ADD_CRC;
                           end

                        6: begin
                           SeqNum_next        = in_data[63:48];
                           LAN_next           = in_data[47:44];
                           LSDU_next          = in_data[43:32];
                           PRP_suffix_next    = in_data[31:16];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[15:0]     = CRC_next[31:16];
                           state_next         = ADD_CRC;
                           end
                     
                        7: begin
                           SeqNum_next        = in_data[55:40];
                           LAN_next           = in_data[39:36];
                           LSDU_next          = in_data[35:24];
                           PRP_suffix_next    = in_data[23:8];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_1           = in_data;
                           output_temp_1[7:0]      = CRC_next[31:24];
                           state_next         = ADD_CRC;
                           end                    
                     endcase
               end 
               output_A = output_temp_1;//                
            end
        end

        ADD_CRC: begin //finish reading packet fields and adds CRC for all scenarios
         if(in_wr) begin
            output_temp_2 = 0;
            if(PRP == 1) begin
               if(src_port == 0) begin //packet from internal CPU, will be modified and duplicated
                  output_temp_2 = 0;
                  if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin
                     case(len_shift_b)
                        0: begin
                           output_temp_2[63:60] = LAN_wr;
                           output_temp_2[59:48] = LSDR_wr;
                           output_temp_2[47:32] = PRP_ID;
                           output_temp_2[31:0]  = CRC;
                           state_next = WAIT_EOP;
                        end

                        1: begin
                           output_temp_2[63:56] = SeqNum_wr[7:0];
                           output_temp_2[55:52] = LAN_wr;
                           output_temp_2[51:40] = LSDR_wr;
                           output_temp_2[39:24] = PRP_ID;
                           output_temp_2[23:0]  = CRC[31:8];
                           state_next = COMPLETE_CRC;
                        end

                        2: begin
                           output_temp_2[63:48] = CRC[15:0];
                           state_next = WAIT_EOP;
                        end

                        3: begin
                           output_temp_2[63:40] = CRC[23:0];
                           state_next = WAIT_EOP;
                        end

                        4: begin
                           output_temp_2[63:32] = CRC;
                           state_next = WAIT_EOP;
                        end

                        5: begin
                           output_temp_2[63:56] = PRP_ID[7:0];
                           output_temp_2[55:24] = CRC;
                           state_next = WAIT_EOP;
                        end

                        6: begin
                           output_temp_2[63:48] = PRP_ID;
                           output_temp_2[47:16] = CRC;
                           state_next = WAIT_EOP;
                        end

                        7: begin
                           output_temp_2[63:56] = LSDR_wr[7:0];
                           output_temp_2[55:40] = PRP_ID;
                           output_temp_2[39:8]  = CRC;
                           state_next = WAIT_EOP;
                        end
                     endcase
                  end
                  else begin 
                     case(len_shift_b)
                        0: begin
                           output_temp_2[63:32] = CRC;
                           state_next = WAIT_EOP;
                           end

                        1: begin
                           output_temp_2[63:56] = PRP_ID[7:0];
                           output_temp_2[55:24] = CRC;
                           state_next = WAIT_EOP;
                           end

                        2: begin
                           output_temp_2[63:48] = PRP_ID;
                           output_temp_2[47:16] = CRC;
                           state_next = WAIT_EOP;
                        end

                        3: begin
                           output_temp_2[63:56] = LSDR_wr[7:0];
                           output_temp_2[55:40] = PRP_ID;
                           output_temp_2[39:8]  = CRC;
                           state_next = WAIT_EOP;
                        end

                        4: begin
                           output_temp_2[63:60] = LAN_wr;
                           output_temp_2[59:48] = LSDR_wr;
                           output_temp_2[47:32] = PRP_ID;
                           output_temp_2[31:0]  = CRC;
                           state_next = WAIT_EOP;
                        end

                        5: begin
                           output_temp_2[63:56] = SeqNum_wr[7:0];
                           output_temp_2[55:52] = LAN_wr;
                           output_temp_2[51:40] = LSDR_wr;
                           output_temp_2[39:24] = PRP_ID;
                           output_temp_2[23:0]  = CRC[31:8];
                           state_next = COMPLETE_CRC;
                        end

                        6: begin
                           output_temp_2[63:48] = CRC[15:0];
                           state_next = WAIT_EOP;
                        end

                        7: begin
                           output_temp_2[63:40] = CRC[23:0];
                           state_next = WAIT_EOP;
                        end
                     endcase
                  //state_next = CHECK_PRP;
                  end
                  output_A_FINAL = output_temp_2;
                  output_B_FINAL = output_temp_2;
               end   
               else begin //packet from LAN might be PRP or SAN
                  if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin
                     case(len_shift_b)                      
                        0: begin
                           LAN_next           = in_data[63:60];
                           LSDU_next          = in_data[59:48];
                           PRP_suffix_next    = in_data[47:32];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_2           = in_data;
                           output_temp_2[31:0]     = CRC_next;
                           state_next         = CHECK_PRP;
                           end

                        1: begin
                           SeqNum_next[7:0]   = in_data[63:56];
                           LAN_next           = in_data[55:52];
                           LSDU_next          = in_data[51:40];
                           PRP_suffix_next    = in_data[39:24];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_2           = in_data;
                           output_temp_2[23:0]     = CRC_next[31:8];
                           state_next         = COMPLETE_CRC;
                           end

                        2: begin
                           output_temp_2[15:0]     = CRC[15:0];
                           state_next = CHECK_PRP;
                           end

                        3: begin
                           output_temp_2[63:40]     = CRC[23:0];
                           state_next = CHECK_PRP;
                           end

                        4: begin
                           output_temp_2[63:32]     = CRC;
                           state_next = CHECK_PRP;
                           end

                        5: begin 
                           PRP_suffix_next[7:0]    = in_data[63:56];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_2           = in_data;
                           output_temp_2[55:24]    = CRC_next;
                           state_next = CHECK_PRP;
                           end

                        6: begin 
                           PRP_suffix_next    = in_data[63:48];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_2           = in_data;
                           output_temp_2[47:16]    = CRC_next;
                           state_next = CHECK_PRP;
                           end

                        7: begin 
                           LSDU_next[7:0]     = in_data[63:56];
                           PRP_suffix_next    = in_data[55:40];
                           CRC_next           = nextCRC32_D64(in_data, CRC);
                           output_temp_2           = in_data;
                           output_temp_2[39:8]     = CRC_next;
                           state_next = CHECK_PRP;
                           end
                     endcase
                  end
                  else begin 
                     case(len_shift_b)
                           0: begin
                              output_temp_2[63:31]     = CRC_next;
                              state_next = CHECK_PRP;
                              end

                           1: begin 
                              PRP_suffix_next[7:0]    = in_data[63:55];
                              CRC_next           = nextCRC32_D64(in_data, CRC);
                              output_temp_2           = in_data;
                              output_temp_2[54:23]    = CRC_next;
                              state_next = CHECK_PRP;
                              end

                           2: begin 
                              PRP_suffix_next    = in_data[63:48];
                              CRC_next           = nextCRC32_D64(in_data, CRC);
                              output_temp_2           = in_data;
                              output_temp_2[47:16]    = CRC_next;
                              state_next = CHECK_PRP;
                              end

                           3: begin 
                              LSDU_next[7:0]     = in_data[63:56];
                              PRP_suffix_next    = in_data[55:40];
                              CRC_next           = nextCRC32_D64(in_data, CRC);
                              output_temp_2           = in_data;
                              output_temp_2[39:8]     = CRC_next;
                              state_next = WAIT_EOP; //CHECK_PRP;
                              end

                           4: begin
                              LAN_next           = in_data[63:60];
                              LSDU_next          = in_data[59:48];
                              PRP_suffix_next    = in_data[47:32];
                              CRC_next           = nextCRC32_D64(in_data, CRC);
                              output_temp_2           = in_data;
                              output_temp_2[31:0]     = CRC_next;
                              state_next         = CHECK_PRP;
                              end

                           5: begin
                              SeqNum_next[7:0]   = in_data[63:56];
                              LAN_next           = in_data[55:52];
                              LSDU_next          = in_data[51:40];
                              PRP_suffix_next    = in_data[39:24];
                              CRC_next           = nextCRC32_D64(in_data, CRC);
                              output_temp_2           = in_data;
                              output_temp_2[23:0]     = CRC_next[31:8];
                              state_next         = COMPLETE_CRC;
                              end

                           6: begin
                              output_temp_2[15:0]     = CRC[15:0];
                              state_next = CHECK_PRP;
                              end

                           7: begin
                              output_temp_2[63:40]     = CRC[23:0];
                              state_next = CHECK_PRP;
                              end   
                     endcase
                  end
               end
               output_A = output_temp_2;//
            end 
            else begin //Non prp mode - only CRC will be added after payload
               if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin 
                  case(len_shift_b) 
                     0: begin
                        dst_port_next      = in_data[15:12];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[11:0]     = CRC_next[31:20];
                        state_next         = COMPLETE_CRC;
                        end
                  
                     1: begin
                        dst_port_next      = in_data[7:4];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[3:0]      = CRC_next[31:28];
                        state_next         = COMPLETE_CRC;
                        end
                     
                     2: begin
                        dst_port_next      = in_data[63:60];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[59:28]     = CRC_next;
                        state_next         = WAIT_EOP;
                        end
                  
                     3: begin
                        dst_port_next      = in_data[55:52];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[51:20]      = CRC_next;
                        state_next         = WAIT_EOP;
                        end

                     4: begin
                        dst_port_next      = in_data[47:44];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[43:12]    = CRC_next;
                        state_next         = WAIT_EOP;
                        end

                     5: begin
                        dst_port_next      = in_data[39:36];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[35:4]     = CRC_next;
                        state_next         = WAIT_EOP;
                        end

                     6: begin
                        dst_port_next      = in_data[31:28];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[27:0]     = CRC_next[31:4];
                        state_next         = COMPLETE_CRC;
                        end

                     7: begin
                        dst_port_next      = in_data[23:20];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[19:0]     = CRC_next[31:12];
                        state_next         = COMPLETE_CRC;
                        end
                  endcase
               end
               else begin
                  case(len_shift_b) 
                     0: begin
                        dst_port_next      = in_data[47:44];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[43:12]    = CRC_next;
                        state_next         = WAIT_EOP;
                        end
                  
                     1: begin
                        dst_port_next      = in_data[39:36];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[35:4]     = CRC_next;
                        state_next         = WAIT_EOP;
                        end
                     
                     2: begin
                        dst_port_next      = in_data[31:28];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[27:0]     = CRC_next[31:4];
                        state_next         = COMPLETE_CRC;
                        end

                     3: begin
                        dst_port_next      = in_data[23:20];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[19:0]     = CRC_next[31:12];
                        state_next         = COMPLETE_CRC;
                        end

                     4: begin
                        dst_port_next      = in_data[15:12];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[11:0]     = CRC_next[31:20];
                        state_next         = COMPLETE_CRC;
                        end

                     5: begin
                        dst_port_next      = in_data[7:4];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[3:0]      = CRC_next[31:28];
                        state_next         = COMPLETE_CRC;
                        end

                     6: begin
                        dst_port_next      = in_data[63:60];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[59:28]     = CRC_next;
                        state_next         = WAIT_EOP;
                        end

                     7: begin
                        dst_port_next      = in_data[55:52];
                        CRC_next           = nextCRC32_D64(in_data, CRC);
                        output_temp_1           = in_data;
                        output_temp_1[51:20]      = CRC_next;
                        state_next         = WAIT_EOP;
                        end                    
                  endcase
               end
               output_A_FINAL = output_temp_1;
               output_B_FINAL = output_temp_1;
            end
        end
        end

        COMPLETE_CRC: begin //finish adding CRC
         if(in_wr) begin
            if(PRP == 1) begin
               output_temp_3 = 0;
               if(src_port == 0) begin
                  if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin
                     output_temp_3[63:56] = CRC[7:0];
                  end
                  else begin
                     output_temp_3[63:56] = CRC[7:0];
                  end 
                  output_A_FINAL = output_temp_3;
                  output_B_FINAL = output_temp_3;
                  state_next = WAIT_EOP;
               end
               else begin
                  if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin
                     output_temp_3[63:56] = CRC[7:0];
                  end
                  else begin
                     output_temp_3[63:56] = CRC[7:0];
                  end
                  state_next = CHECK_PRP;
               end
            end
            else begin
               output_temp_2 = in_data;
               if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin
                  case(len_shift_b)
                  0: begin
                     output_temp_2[63:44] = CRC[19:0];
                     state_next = WAIT_EOP;
                  end

                  1: begin
                     output_temp_2[63:36] = CRC[27:0];
                     state_next = WAIT_EOP;
                  end

                  6: begin
                     output_temp_2[63:60] = CRC[3:0];
                     state_next = WAIT_EOP;
                  end

                  7: begin
                     output_temp_2[63:52] = CRC[11:0];
                     state_next = WAIT_EOP;
                  end
                  endcase
               end
               else begin
                  case(len_shift_b)
                  4: begin
                     output_temp_2[63:44] = CRC[19:0];
                     state_next = WAIT_EOP;
                  end

                  5: begin
                     output_temp_2[63:36] = CRC[27:0];
                     state_next = WAIT_EOP;
                  end

                  2: begin
                     output_temp_2[63:60] = CRC[3:0];
                     state_next = WAIT_EOP;
                  end


                  3: begin
                     output_temp_2[63:52] = CRC[11:0];
                     state_next = WAIT_EOP;
                  end
                  endcase
               end
               output_A_FINAL = output_temp_2;
               output_B_FINAL = output_temp_2;
            end 
        end  
        end   

        CHECK_PRP: begin //when packet comes from LAN check if it's PRP packet or from SAN
           if(PRP_suffix == PRP_ID) begin
              case(PRP_words)
               0: begin
                  output_A_FINAL = output_temp_1;
                  output_B_FINAL = output_temp_1;
               end 
               1: begin
                  output_A_FINAL = output_temp_2;
                  output_B_FINAL = output_temp_2;
               end
               2: begin
                  output_A_FINAL = output_temp_3;
                  output_B_FINAL = output_temp_3;
               end
               3: begin
                  PRP_words = 0;
                  state_next = WAIT_EOP;
               end
              endcase
            PRP_words = PRP_words + 1;  
           end
           else begin
              case(SAN_words)
                  1: begin
                  if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin 
                     case(len_shift_b) 
                        0: begin
                           dst_port_next      = output_temp_1[15:12];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[11:0]     = CRC[31:20];
                           output_B_FINAL[11:0]     = CRC[31:20];
                           end
                     
                        1: begin
                           dst_port_next      = output_temp_1[7:4];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[3:0]     = CRC[31:28];
                           output_B_FINAL[3:0]     = CRC[31:28];
                           end
                        
                        2: begin
                           dst_port_next      = output_temp_1[63:60];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[59:28]     = CRC;
                           output_B_FINAL[59:28]     = CRC;
                           end
                     
                        3: begin
                           dst_port_next      = output_temp_1[55:52];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[51:20]     = CRC;
                           output_B_FINAL[51:20]     = CRC;
                           end

                        4: begin
                           dst_port_next      = output_temp_1[47:44];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[43:12]     = CRC;
                           output_B_FINAL[43:12]     = CRC;
                           end

                        5: begin
                           dst_port_next      = output_temp_1[39:36];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[35:4]     = CRC;
                           output_B_FINAL[35:4]     = CRC;
                           end

                        6: begin
                           dst_port_next      = output_temp_1[31:28];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[27:0]     = CRC[31:4];
                           output_B_FINAL[27:0]     = CRC[31:4];
                           end

                        7: begin
                           dst_port_next      = output_temp_1[23:20];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[19:0]     = CRC[31:12];
                           output_B_FINAL[19:0]     = CRC[31:12];
                           end
                     endcase
                  end
                  else begin
                     case(len_shift_b) 
                        0: begin
                           dst_port_next      = in_data[47:44];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[43:12]     = CRC;
                           output_B_FINAL[43:12]     = CRC;
                           end
                     
                        1: begin
                           dst_port_next      = in_data[39:36];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[35:4]     = CRC;
                           output_B_FINAL[35:4]     = CRC;
                           end
                        
                        2: begin
                           dst_port_next      = in_data[31:28];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[27:0]     = CRC[31:4];
                           output_B_FINAL[27:0]     = CRC[31:4];
                           end

                        3: begin
                           dst_port_next      = in_data[23:20];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[19:0]     = CRC[31:12];
                           output_B_FINAL[19:0]     = CRC[31:12];
                           end

                        4: begin
                           dst_port_next      = in_data[15:12];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[11:0]     = CRC[31:20];
                           output_B_FINAL[11:0]     = CRC[31:20];
                           end

                        5: begin
                           dst_port_next      = in_data[7:4];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[3:0]     = CRC[31:28];
                           output_B_FINAL[3:0]     = CRC[31:28];
                           end

                        6: begin
                           dst_port_next      = in_data[63:60];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1;
                           output_A_FINAL[59:28]     = CRC;
                           output_B_FINAL[59:28]     = CRC;
                           end

                        7: begin
                           dst_port_next      = in_data[55:52];
                           output_A_FINAL     = output_temp_1;
                           output_B_FINAL     = output_temp_1; 
                           output_A_FINAL[51:20]     = CRC;
                           output_B_FINAL[51:20]     = CRC;  
                           end                    
                     endcase
                     end
                  end

                  2: begin
                     output_A_FINAL = 0;
                     output_B_FINAL = 0;
                     if(((VLANheaders==2 || VLANheaders==0) && eth_done_next==0) || (VLANheaders==1 && eth_done_next==1)) begin
                        case(len_shift_b)
                           0: begin
                              output_A_FINAL[63:44] = CRC[19:0];
                              output_B_FINAL[63:44] = CRC[19:0];
                           end

                           1: begin
                              output_A_FINAL[63:40] = CRC[27:0];
                              output_B_FINAL[63:40] = CRC[27:0];
                           end

                           6: begin
                              output_A_FINAL[63:60] = CRC[3:0];
                              output_B_FINAL[63:60] = CRC[3:0];
                           end

                           7: begin
                              output_A_FINAL[63:42] = CRC[11:0];
                              output_B_FINAL[63:42] = CRC[11:0];
                           end
                        endcase
                     end
                     else begin 
                        case(len_shift_b)
                           2: begin
                              output_A_FINAL[63:60] = CRC[4:0];
                              output_B_FINAL[63:60] = CRC[4:0];
                           end

                           3: begin
                              output_A_FINAL[63:52] = CRC[11:0];
                              output_B_FINAL[63:52] = CRC[11:0];
                           end

                           4: begin
                              output_A_FINAL[63:44] = CRC[19:0];
                              output_B_FINAL[63:44] = CRC[19:0];
                           end

                           5: begin
                              output_A_FINAL[63:40] = CRC[27:0];
                              output_B_FINAL[63:40] = CRC[27:0];
                           end
                        endcase
                     end
                  end

                  3: begin
                     state_next = WAIT_EOP;
                     SAN_words = 1;
                  end
              endcase
              SAN_words = SAN_words + 1;
           end
        end

        WAIT_EOP: begin //wait EOP and insert MAC+SN on hash table if PRP is on
           if(in_wr && in_ctrl!=0) begin
               eth_done_next   = 0;
               state_next      = READ_WORD_1;
               counter         = 0;

               if(PRP == 1) begin 
                  if(src_port == 0) //packet from CPU - duplicate and insert
                  begin
                     ht_cmd_in.cmd[1:0] = 2'b01; //cmd = 1 insert
                     ht_cmd_in.cmd.value = 1;
                     ht_cmd_in.cmd.key[47:8] = src_mac[40:0];
                     ht_cmd_in.cmd.key[7:0] = SeqNum_wr[7:0];
                     ht_cmd_in.valid = 1;
                  end
                  else
                  begin //packet from LAN - check for duplicate
                     ht_cmd_in.cmd[1:0] = 2'b00; //cmd = 0 search
                     ht_cmd_in.cmd.value = 1;
                     ht_cmd_in.cmd.key[47:8] = src_mac[40:0];
                     ht_cmd_in.cmd.key[7:0] = SeqNum_wr[7:0];
                     ht_cmd_in.valid = 1;
                     state_next = WAIT_RESULT;
                  end
               end  
               output_A_FINAL = 0;
               output_B_FINAL = 0;
           end  
        end

       WAIT_RESULT: begin
          if(ht_res_out.valid)
            begin
               if(ht_res_out.result.rescode == SEARCH_FOUND)
                  not_found_table = 0;
               else
                  not_found_table = 1;

               state_next = READ_WORD_1;   
            end
       end

      endcase // case(state)
   end // always @ (*)

   always @(posedge clk) begin      if(reset) begin
         src_mac      <= 0;
         dst_mac      <= 0;
         ethertype    <= 0;
         eth_done     <= 0;
         state        <= READ_WORD_1;
         src_port     <= 0;
         dst_port     <= 0;
         TPID1        <= 0;
         TPID2        <= 0;
         TCI1         <= 0;
         TCI2         <= 0;
         length       <= 0;
         IHL          <= 0;
         len_shift_w  <= 0;
         len_shift_b  <= 0;
         IHL_aux      <= 0;  
         length_aux   <= 0;
         counter      <= 0;
         LAN          <= 0;
         shift32      <= 0;
         CRC          <= 0;
         SeqNum       <= 0;
         LSDU         <= 0;
         PRP_suffix   <= 0;
         ht_res_out.ready <= 0;
         ht_cmd_in.valid <= 0;
         ht_cmd_in.cmd <= 0;
         PRP_words    <= 0;
         SAN_words    <= 1;
         SeqNum_wr     <= 0;
         header_words <= 0;
      end
      else begin
         src_mac      <= src_mac_next;
         dst_mac      <= dst_mac_next;
         ethertype    <= ethertype_next;
         eth_done     <= eth_done_next;
         state        <= state_next;
         src_port     <= src_port_next;
         dst_port     <= dst_port_next;
         TPID1        <= TPID1_next;
         TPID2        <= TPID2_next;
         TCI1         <= TCI1_next;
         TCI2         <= TCI2_next;
         length       <= length_next;
         IHL          <= IHL_next;
         LAN          <= LAN_next;
         CRC          <= CRC_next;
         SeqNum       <= SeqNum_next;
         LSDU         <= LSDU_next;
         PRP_suffix   <= PRP_suffix_next;
         header_words <= header_words_next;

         if(state_next == SHIFT_WORDS) begin
            counter = counter +1;
         end

         if(state_next == ADD_PRP_tailer) begin
            SeqNum_wr = SeqNum_wr + 1;
         end

         if(state_next <= WAIT_EOP /*READ_PRP_tailer && state_next != READ_WORD_1*/) begin
            output_A_FINAL = output_A;
            output_B_FINAL = output_A;
         end
      end // else: !if(reset)
   end // always @ (posedge clk)

endmodule // ethernet_parser_64bit
