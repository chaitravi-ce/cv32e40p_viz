\m5_TLV_version 1d: tl-x.org
   
\m5
   // =================================================
   // Welcome!  New to Makerchip? Try the "Learn" menu.
   // =================================================
   use(m5-1.0)   /// uncomment to use M5 macro library.
\SV
   // URL include paths:
   m4_define(['m4_cv32e40p_repo'], ['['https://raw.githubusercontent.com/chaitravi-ce/cv32e40p/master/']'])
   m4_define(['m4_cv32e40p_src'], ['m4_cv32e40p_repo['rtl/']'])
   m4_define(['m4_cv32e40p_cells_src'], ['m4_cv32e40p_repo['rtl/vendor/pulp_platform_common_cells/include/common_cells/']'])                                   
   m4_define(['m4_cv32e40p_fpnew_src'], ['m4_cv32e40p_repo['rtl/vendor/pulp_platform_fpnew/src/']'])  
   m4_define(['m4_cv32e40p_verif_repo'], ['['https://raw.githubusercontent.com/shariethernet/viz_corevverif/main/cv32e40p/']'])
   m4_define(['m4_cv32e40p_verif_src'], ['m4_cv32e40p_verif_repo['tb/core/']'])  
   m4_define(['m4_assets_repo'], ['['https://raw.githubusercontent.com/chaitravi-ce/makerchip_assets/main/']'])                                         
                                      
                                      
   m4_sv_get_url(m4_cv32e40p_src['include/cv32e40p_apu_core_pkg.sv'])
   m4_sv_get_url(m4_cv32e40p_src['include/cv32e40p_fpu_pkg.sv'])
   m4_sv_get_url(m4_cv32e40p_src['include/cv32e40p_pkg.sv'])
                                    //https://raw.githubusercontent.com/chaitravi-ce/cv32e40p/a391b0a0dbd49da79fa5faf43a7e55742f26f5e4/rtl/cv32e40p_core.sv  
   // Headers:
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_fp_wrapper.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_core.sv '])  
// m4_sv_get_url(m4_cv32e40p_src['cv32e40p_core.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_aligner.sv'])
   m4_sv_get_url(['https://raw.githubusercontent.com/chaitravi-ce/cv32e40p/master/bhv/cv32e40p_clock_gate.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_alu.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_alu_div.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_apu_disp.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_compressed_decoder.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_controller.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_cs_registers.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_decoder.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_ex_stage.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_ff_one.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_fifo.sv'])                                    
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_hwloop_regs.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_id_stage.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_if_stage.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_int_controller.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_load_store_unit.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_mult.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_obi_interface.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_popcnt.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_prefetch_buffer.sv'])
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_prefetch_controller.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_register_file.sv'])                                  
   m4_sv_get_url(m4_cv32e40p_src['cv32e40p_sleep_unit.sv'])  
                                      
   m4_sv_include_url(m4_cv32e40p_cells_src['registers.svh'], ['common_cells/registers.svh'])
   m4_sv_include_url(m4_cv32e40p_cells_src['assertions.svh'], ['common_cells/assertions.svh'])                                   
                                      
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_top.sv'])    
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_pkg.sv'])                                     
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_rounding.sv'])    
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_opgroup_multifmt_slice.sv'])                                   
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_opgroup_fmt_slice.sv'])    
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_opgroup_block.sv'])                                     
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_noncomp.sv'])    
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_fma_multi.sv'])   
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_fma.sv'])    
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_divsqrt_multi.sv'])                                     
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_classifier.sv'])    
   m4_sv_get_url(m4_cv32e40p_fpnew_src['fpnew_cast_multi.sv'])  
                                            
                                            
   m4_sv_get_url(m4_cv32e40p_verif_src['mm_ram.sv']) 
   m4_sv_get_url(m4_cv32e40p_verif_src['dp_ram.sv'])  
   m4_sv_get_url(m4_cv32e40p_verif_src['tb_riscv/riscv_rvalid_stall.sv'])
   m4_sv_get_url(m4_cv32e40p_verif_src['tb_riscv/riscv_gnt_stall.sv'])
   m4_sv_get_url(m4_cv32e40p_verif_src['tb_riscv/riscv_perturbation.sv']) 
   m4_sv_get_url(m4_cv32e40p_verif_src['tb_riscv/riscv_random_interrupt_generator.sv'])
   m4_sv_get_url(m4_cv32e40p_verif_src['tb_riscv/riscv_random_stall.sv'])
   m4_sv_get_url(m4_cv32e40p_verif_src['tb_riscv/riscv_simchecker.sv']) 
      
  //m4_sv_get_url(m4_cv32e40p_verif_src['tb_top.sv']) 
   m4_sv_get_url(m4_cv32e40p_verif_src['tb_top_verilator.sv'])  
                                    
   m4_sv_get_url(m4_assets_repo['hello-world.hex']) 
                   
   m5_var(DATA_WIDTH, 32)
   m5_var(ADDR_WIDTH, 5)
   m5_var(RAM_WIDTH, 8)
   m5_var(NUM_INSTRS, 32)
   /* verilator lint_off BLKANDNBLK */
   /* verilator lint_off IMPLICIT */
   /* verilator lint_off ASCRANGE */
 /* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
    `pragma verilator lint_off UNOPTFLAT
    `pragma verilator line_off LATCH
    `pragma verilator lint_off MODDUP
    `pragma verilator lint_off lint
                  module cv32e40p_tb_wrapper
    #(parameter // Parameters used by TB
                INSTR_RDATA_WIDTH = 32,
                RAM_ADDR_WIDTH    = 20,
                BOOT_ADDR         = 'h80,
                DM_HALTADDRESS    = 32'h1A11_0800,
                HART_ID           = 32'h0000_0000,
                // Parameters used by DUT
                PULP_XPULP        = 0,
                PULP_CLUSTER      = 0,
                FPU               = 0,
                PULP_ZFINX        = 0,
                NUM_MHPMCOUNTERS  = 1
    )
    (input logic         clk_i,
     input logic         reset,
     input logic         fetch_enable_i,
     output logic        tests_passed_o,
     output logic        tests_failed_o,
     output logic [31:0] exit_value_o,
     output logic        exit_valid_o);
    // signals connecting core to memory
    logic rst_ni;
    assign rst_ni = ~reset;
    logic                         instr_req;
    logic                         instr_gnt;
    logic                         instr_rvalid;
    logic [31:0]                  instr_addr;
    logic [INSTR_RDATA_WIDTH-1:0] instr_rdata;
    logic                         data_req;
    logic                         data_gnt;
    logic                         data_rvalid;
    logic [31:0]                  data_addr;
    logic                         data_we;
    logic [3:0]                   data_be;
    logic [31:0]                  data_rdata;
    logic [31:0]                  data_wdata;
    // signals to debug unit
    logic                         debug_req;
    // irq signals (not used)
    logic [0:31]                  irq;
    logic [0:4]                   irq_id_in;
    logic                         irq_ack;
    logic [0:4]                   irq_id_out;
    logic                         irq_sec;
    // interrupts (only timer for now)
    assign irq_sec     = '0;
    // instantiate the core
     /* verilator lint_off PINMISSING */
    cv32e40p_core #(
                 .COREV_PULP       (PULP_XPULP),
                 .COREV_CLUSTER     (PULP_CLUSTER),
                 .FPU              (FPU),
                 .ZFINX       (PULP_ZFINX),
                 .NUM_MHPMCOUNTERS (NUM_MHPMCOUNTERS)
                )
    cv32e40p_core_i
        (
         .clk_i                  ( clk_i                 ),
         .rst_ni                 ( rst_ni                ),
         .pulp_clock_en_i        ( '1                    ),
         .scan_cg_en_i           ( '0                    ),
         .boot_addr_i            ( BOOT_ADDR             ),
         .dm_halt_addr_i         ( DM_HALTADDRESS        ),
         .hart_id_i              ( HART_ID               ),
         .instr_req_o            ( instr_req             ),
         .instr_gnt_i            ( instr_gnt             ),
         .instr_rvalid_i         ( instr_rvalid          ),
         .instr_addr_o           ( instr_addr            ),
         .instr_rdata_i          ( instr_rdata           ),
         .data_req_o             ( data_req              ),
         .data_gnt_i             ( data_gnt              ),
         .data_rvalid_i          ( data_rvalid           ),
         .data_we_o              ( data_we               ),
         .data_be_o              ( data_be               ),
         .data_addr_o            ( data_addr             ),
         .data_wdata_o           ( data_wdata            ),
         .data_rdata_i           ( data_rdata            ),
         .apu_req_o              (                       ),
         .apu_gnt_i              ( 1'b0                  ),
         .apu_operands_o         (                       ),
         .apu_op_o               (                       ),
         .apu_flags_o            (                       ),
         .apu_rvalid_i           ( 1'b0                  ),
         .apu_result_i           ( {32{1'b0}}            ),
         .apu_flags_i            ( {5{1'b0}}             ), // APU_NUSFLAGS_CPU
         // Interrupts verified in UVM environment
         .irq_i                  ( {32{1'b0}}            ),
         .irq_ack_o              ( irq_ack               ),
         .irq_id_o               ( irq_id_out            ),
         .debug_req_i            ( debug_req             ),
         .fetch_enable_i         ( fetch_enable_i        ),
         .core_sleep_o           ( core_sleep_o          )
       );
    // this handles read to RAM and memory mapped pseudo peripherals
    mm_ram
        #(.RAM_ADDR_WIDTH (RAM_ADDR_WIDTH),
          .INSTR_RDATA_WIDTH (INSTR_RDATA_WIDTH))
    ram_i
        (.clk_i          ( clk_i                                     ),
         .rst_ni         ( rst_ni                                    ),
         .dm_halt_addr_i ( DM_HALTADDRESS                            ),
         .instr_req_i    ( instr_req                                 ),
         .instr_addr_i   ( { {10{1'b0}},
                             instr_addr[RAM_ADDR_WIDTH-1:0]
                           }                                         ),
         .instr_rdata_o  ( instr_rdata                               ),
         .instr_rvalid_o ( instr_rvalid                              ),
         .instr_gnt_o    ( instr_gnt                                 ),
         .data_req_i     ( data_req                                  ),
         .data_addr_i    ( data_addr                                 ),
         .data_we_i      ( data_we                                   ),
         .data_be_i      ( data_be                                   ),
         .data_wdata_i   ( data_wdata                                ),
         .data_rdata_o   ( data_rdata                                ),
         .data_rvalid_o  ( data_rvalid                               ),
         .data_gnt_o     ( data_gnt                                  ),
         .irq_id_i       ( irq_id_out                                ),
         .irq_ack_i      ( irq_ack                                   ),
         .irq_o          ( irq                                       ),
         .debug_req_o    ( debug_req                                 ),
         .pc_core_id_i   ( cv32e40p_core_i.pc_id                     ),
         .tests_passed_o ( tests_passed_o                            ),
         .tests_failed_o ( tests_failed_o                            ),
         .exit_valid_o   ( exit_valid_o                              ),
         .exit_value_o   ( exit_value_o                              ));
   endmodule                                 
                                    
     //-- Main TB starts here--                               
   m5_makerchip_module               
   
   logic tests_passed_o;
   logic tests_failed_o;
   logic fetch_enable_i;
   logic[31:0] instr_rdata; 
                  
   logic if_stage;
   logic id_stage;   
   logic ex_stage;              
   logic wb_stage;               
                   
   // number of integer registers
   localparam NUM_WORDS = 2 ** (m5_ADDR_WIDTH - 1);
   localparam bytes = 2**m5_RAM_WIDTH;
   
   // integer register file
   logic [NUM_WORDS-1:0][m5_DATA_WIDTH-1:0] mem;
                   
   // fp register file
   logic [ NUM_WORDS-1:0][m5_DATA_WIDTH-1:0] mem_fp;
                   
   logic [7:0] dp_ram[bytes];
                  
   // csr registers
   logic [31:0] fcsr;
                  
   //fifo
   logic [1:0] fifo_cnt;
   logic [31:0] fifo_rdata;
   logic fifo_push;
   logic fifo_pop;
                  
   //interfaces               
   logic instr_rvalid;     
   logic [31:0] instr_addr; 
   logic data_gnt;
   logic [31:0] data_addr;           
   logic data_rvalid;         
                  
   //if stage
   logic [31:0] if_stage_fetch_rdata;
   logic if_stage_if_valid;
   logic if_stage_id_ready;        
                  
   //LSU
   logic [31:0] lsu_data_addr;
   logic lsu_data_gnt;
   logic lsu_data_we;
   logic [31:0] lsu_data_wdata;
   logic [31:0] lsu_data_rdata;
   logic lsu_data_rvalid;
             
   assign instr_rdata = cv32e40p_tb_wrapper_i.cv32e40p_core_i.instr_rdata_i;
                  
   assign if_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.if_valid;               
   assign id_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.id_valid;
   assign ex_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.ex_valid;
   assign wb_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.wb_valid;
                   
   assign mem = cv32e40p_tb_wrapper_i.cv32e40p_core_i.id_stage_i.register_file_i.mem; 
   assign mem_fp = cv32e40p_tb_wrapper_i.cv32e40p_core_i.id_stage_i.register_file_i.mem_fp;
     
   assign fcsr = cv32e40p_tb_wrapper_i.cv32e40p_core_i.cs_registers_i.fcsr_update;
                  
   assign fifo_cnt = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.prefetch_buffer_i.fifo_cnt[1:0];
   assign fifo_rdata = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.prefetch_buffer_i.fetch_rdata_o[31:0];
   assign fifo_push = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.prefetch_buffer_i.fifo_push;
   assign fifo_pop = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.prefetch_buffer_i.fifo_pop;               
   
   assign if_stage_fetch_rdata = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.fetch_rdata[31:0]; 
   assign if_stage_if_valid = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.if_valid;         
   assign if_stage_id_ready = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.id_ready_i;                      
                  
   assign instr_rvalid = cv32e40p_tb_wrapper_i.cv32e40p_core_i.instr_rvalid_i;                
   assign instr_addr = cv32e40p_tb_wrapper_i.cv32e40p_core_i.instr_addr_o[31:0];
                  
   assign data_gnt = cv32e40p_tb_wrapper_i.cv32e40p_core_i.data_gnt_i;                
   assign data_addr = cv32e40p_tb_wrapper_i.cv32e40p_core_i.data_addr_o[31:0]; 
   assign data_rvalid = cv32e40p_tb_wrapper_i.cv32e40p_core_i.data_rvalid_i;                 
                  
   assign lsu_data_addr = cv32e40p_tb_wrapper_i.cv32e40p_core_i.load_store_unit_i.data_addr_o[31:0];
   assign lsu_data_gnt = cv32e40p_tb_wrapper_i.cv32e40p_core_i.load_store_unit_i.data_gnt_i;
   assign lsu_data_we = cv32e40p_tb_wrapper_i.cv32e40p_core_i.load_store_unit_i.data_we_o;             
   assign lsu_data_rvalid = cv32e40p_tb_wrapper_i.cv32e40p_core_i.load_store_unit_i.data_rvalid_i;                
   assign lsu_data_wdata = cv32e40p_tb_wrapper_i.cv32e40p_core_i.load_store_unit_i.data_wdata[31:0]; 
   assign lsu_data_rdata = cv32e40p_tb_wrapper_i.cv32e40p_core_i.load_store_unit_i.data_rdata_i[31:0];  
                  
   integer i;
   initial begin
     for (i = 0; i < NUM_WORDS; i++) begin
       $display("mem[%0d] = %h", i, mem[i][31:0]);
       $display("mem_fp[%0d] = %h", i, mem_fp[i][31:0]);
     end
   end
                   
   logic [m5_NUM_INSTRS-1:0][m5_DATA_WIDTH-1:0] instrs;
                   
   assign instrs = '{
      32'b101000000010010011,
      32'b1111111101011111010110010011,
      32'b1000000001100111,
      32'b101101110000000000100011,
      32'b101101110000000010100011,
      32'b101101110000000100100011,
      32'b101101110000000110100011,
      32'b101000000010010011,
      32'b11111010000001101000000011100111,
      32'b1000001010010011,
      32'b10101101000011010110011,
      32'b1010010111,
      32'b1001111001011010010011,
      32'b11111111000001100111011010010011,
      32'b1000000001011001001001100011,
      32'b1010000001111001000001100011,
      32'b111101110111011110010011,
      32'b10110000110111111001100011,
      32'b1010000011100010011,
      32'b111100000000001100010011,
      32'b101000101110010101101010001,
      32'b1000101100000011000111000001001,
      32'b100000100000011000011000010011,
      32'b11100110000011000010100010011,
      32'b110000010101010001000001110011,
      32'b101010110010100010011,
      32'b11110111000001010000010100010011,
      32'b10100010111,
      32'b11110111100000010000000100010011,
      32'b10000000000000100010111,
      32'b10110110000000011000000110010011,
      32'b10110000110010111
   };
                  
   int unsigned            cycle_cnt_q;
                  
  
    // testbench result
    logic                   exit_valid;
    logic [31:0]            exit_value; 
                  
   initial begin
      $readmemh("sv_url_inc/hello-world.hex", cv32e40p_tb_wrapper_i.ram_i.dp_ram_i.mem);
      $display("[TESTBENCH] %t: loading firmware ...", $time);
   end
   
   initial begin 
    fetch_enable_i = 1'b1;
   end
                  
   initial begin
      for (i=0; i<8; i++) begin
         assign dp_ram[i] = cv32e40p_tb_wrapper_i.ram_i.dp_ram_i.mem[i];
      end     
   end
     
   always_ff @(posedge clk, negedge (~reset)) begin
        if (tests_passed_o) begin
            $display("%m @ %0t: ALL TESTS PASSED", $time);
            $finish;
        end
        if (tests_failed_o) begin
            $display("%m @ %0t: TEST(S) FAILED!", $time);
            $finish;
        end
        if (exit_valid) begin
            if (exit_value == 0)
                $display("%m @ %0t: EXIT SUCCESS", $time);
            else
                $display("%m @ %0t: EXIT FAILURE: %d", exit_value, $time);
            $finish;
        end
    end
                  
   cv32e40p_tb_wrapper
        #(.INSTR_RDATA_WIDTH (128),
          .RAM_ADDR_WIDTH    (22),
          .BOOT_ADDR         ('h80),
          .PULP_CLUSTER      (0),
          .FPU               (0),
          .PULP_ZFINX        (0),
          .DM_HALTADDRESS    (32'h1A110800)
         )
    cv32e40p_tb_wrapper_i
        (.clk_i          ( clk            ),
         .reset          ( reset          ),
         .fetch_enable_i ( fetch_enable_i ),
         .tests_passed_o ( tests_passed_o ),
         .tests_failed_o ( tests_failed_o ),
         .exit_valid_o   ( exit_valid     ),
         .exit_value_o   ( exit_value     ));
   
   
 /* verilator lint_on PINMISSING */  
 /* verilator lint_on IMPLICIT */
 /* verilator lint_on ASCRANGE */
 /* verilator lint_on WIDTHEXPAND */
 /* verilator lint_on WIDTHTRUNC */
\TLV prefetch_buffer(|_top, _where_)
   /prefetch_buffer
      $fifo_cnt[1:0] = *fifo_cnt;
      $fifo_rdata[31:0] = *fifo_rdata;
      $fifo_push = *fifo_push;
      $fifo_pop = *fifo_pop;
      
      \viz_js
         box: {top: 0, left: 0, width: 550, height: 300, fill: "green", strokeWidth: 0},
         init(){
            //m4_get_url_file(['https://gitlab.com/luplab/rvcodecjs/-/raw/main/core/Config.js'], js_modules)
            //m4_get_url_file(['https://gitlab.com/luplab/rvcodecjs/-/raw/main/core/Constants.js'], js_modules)
            //m4_get_url_file(['https://gitlab.com/luplab/rvcodecjs/-/raw/main/core/Instruction.js'], js_modules)

            this.decoder = m5_js_import_url(['https://gitlab.com/luplab/rvcodecjs/-/raw/main/core/Decoder.js'])
            this.encoder = m5_js_import_url(['https://gitlab.com/luplab/rvcodecjs/-/raw/main/core/Encoder.js'])
            this.instruction = m5_js_import_url(['https://gitlab.com/luplab/rvcodecjs/-/raw/main/core/Instruction.js'])
            this.config = m5_js_import_url(['https://gitlab.com/luplab/rvcodecjs/-/raw/main/core/Config.js'])
            
            this.resolved = false
            this.decoder.then(() => {
               this.resolved = true
            })

            this.config.then(function(result) {
               const isa_cops = result.COPTS_ISA["AUTO"]
            });

            let prefetch_header = new fabric.Text("🗃️ Prefetch Buffer", {
               top: 20,
               left: 100,
               fontSize: 30,
               fontWeight: 800,
               fontFamily: "monospace",
               fill: "black"
            })

            let instr = new fabric.Text("", {
               top: 110,
               left: 50,
               fontSize: 22,
               fontFamily: "monospace",
               fill: "black"
            })

            let fifo = new fabric.Text("Contents in FIFO", {
               top: 150,
               left: 50,
               fontSize: 26,
               fontFamily: "monospace",
               fill: "black"
            })

            let fifo_1 = new fabric.Text("1" ,{
               fontFamily: "monospace",
               fill: "black",
               fontSize: 22,
               top: 200,
               left: 50
            })

            let fifo_2 = new fabric.Text("2" , {
               fontFamily: "monospace",
               fill: "black",
               fontSize: 22,
               top: 220,
               left: 50
            })

            this.fifo_array = [];

            return {prefetch_header, instr, fifo, fifo_1, fifo_2}
         },
         render(){
            let $fifo_cnt = '$fifo_cnt'.asInt().toString()
            let fifo_rdata = '$fifo_rdata'.asInt().toString(16)
            let fifo_push = '$fifo_push'.asInt().toString()
            let fifo_pop = '$fifo_pop'.asInt().toString()
      
            if(fifo_pop == 1){
               this.fifo_array.pop()
            }

            let instr = this.getInitObjects().instr;
      
            let fifo_1 = this.getInitObjects().fifo_1;
            let fifo_2 = this.getInitObjects().fifo_2;
      
            this.instruction.then((result) => {
               const inst = new result.Instruction(fifo_rdata, {
                  ABI: false,
                  ISA: isa_cops
               });
      
               let asm_str = inst.asm.toString();
      
               if(fifo_push == 1){
                  this.fifo_array.push(asm_str)
               }
      
               instr.set({text: asm_str})
               this.global.canvas.renderAll.bind(this.global.canvas)()
            });
      
            for(i=0; i<this.fifo_array.length; i++){
               if(i==0){
                  fifo_1.set({text: this.fifo_array[i].toString()})
               }else{
                  fifo_2.set({text: this.fifo_array[i].toString()})
               }
            }

            console.log(this.fifo_array)

            return [
               new fabric.Text("Number of words in Buffer:   " + $fifo_cnt, {
                  fontFamily: "monospace",
                  fill: "black",
                  fontSize: 22,
                  top: 80,
                  left: 50
               }),
            ]
         },
         unrender(){
            if(this.fifo_array.length == 1){
               this.getContext().initObjects.fifo_2.set({text: ""})
            }
            if(this.fifo_array.length == 0){
               this.getContext().initObjects.fifo_2.set({text: ""})
               this.getContext().initObjects.fifo_1.set({text: ""})
            }
         },
         where: {top: 200, left:-650}

\TLV
   |core
      @0
         \viz_js
            init(){
               let pipe_img = this.newImageFromURL(
                        "https://raw.githubusercontent.com/chaitravi-ce/makerchip_assets/main/CV32E40P_Pipeline.png", 
                        "",
                        {left: 0, top: 0, width: 2000},
               )
         
               return {pipe_img}
            }
         
         m5+prefetch_buffer(|core)
         
         /lsu
            $lsu_data_addr[31:0] = *lsu_data_addr;
            $lsu_data_gnt = *lsu_data_gnt;
            $lsu_data_we = *lsu_data_we;
            $lsu_data_wdata[31:0] = *lsu_data_wdata;
            $lsu_data_rdata[31:0] = *lsu_data_rdata;
            $lsu_data_rvalid = *lsu_data_rvalid;
            
            \viz_js
               box: {top: 0, left: 0, width: 550, height: 300, fill: "green", strokeWidth: 0},
               init(){
                  
               },
               render(){
               
               },
               where: {top: 200, left: 650},
         
         /interfaces
            /instruction
               $instr_rvalid = *instr_rvalid;
               $instr_addr[31:0] = *instr_addr;
            
            /data
               $data_addr[31:0] = *data_addr;
               $data_grant = *data_gnt;
               $data_rvalid = *data_rvalid;
      
      @1
         /if_stage
            $if_stage_fetch_rdata[31:0] = *if_stage_fetch_rdata;
            $if_stage_if_valid = *if_stage_if_valid;
            $if_stage_id_ready = *if_stage_id_ready;
            
      
      @2
         /id_stage
            
      
      @3
         /ex_stage
            
      
      @4
         /wb_stage
            
   *passed = !clk || *tests_passed_o;
   *failed = !clk || *tests_failed_o || *cyc_cnt > 1000000;
   

\SV
   endmodule
   