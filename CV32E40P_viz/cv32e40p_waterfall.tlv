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
             
   assign instr_rdata = cv32e40p_tb_wrapper_i.cv32e40p_core_i.instr_rdata_i;
                  
   assign if_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.if_stage_i.if_valid;               
   assign id_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.id_valid;
   assign ex_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.ex_valid;
   assign wb_stage = cv32e40p_tb_wrapper_i.cv32e40p_core_i.wb_valid;
                   
   assign mem = cv32e40p_tb_wrapper_i.cv32e40p_core_i.id_stage_i.register_file_i.mem; 
   assign mem_fp = cv32e40p_tb_wrapper_i.cv32e40p_core_i.id_stage_i.register_file_i.mem_fp;
     
   assign fcsr = cv32e40p_tb_wrapper_i.cv32e40p_core_i.cs_registers_i.fcsr_update;
                  
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
               

\TLV
   |core
      @0
         /instructions
            $instr_rdata[31:0] = *instr_rdata;
            \viz_js
               box: {strokeWidth: 0},
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
                   
                   let imem_header = new fabric.Text("🗃️ Instr. Memory", {
                        top: 10,
                        left: 200,
                        fontSize: 20,
                        fontWeight: 800,
                        fontFamily: "monospace",
                        fill: "black",
                        zIndex: 2
                   })
                   
                   let mem_bg = new fabric.Rect({
                           width: 700,
                           height: 1000,
                           fill: "green",
                           zIndex: -1,
                           left: -50,
                           top: -10
                   });
                   
                   let inst_bg = new fabric.Rect({
                           width: 500,
                           height: 300,
                           fill: "orange",
                           opacity: 0.3,
                           zIndex: -1,
                           left: 680,
                           top: -10,
                           stroke: "brown",
                           strokeWidth: 4
                   });
            
                   let inst = new fabric.Text("", {
                           left: 800,
                           top: 100,
                           fontFamily: "monospace",
                           fill: "black",
                           fontSize: 25
                   });
                   
                   let inst_type = new fabric.Text("", {
                           left: 870,
                           top: 130,
                           fontFamily: "monospace",
                           fill: "black",
                           fontSize: 18
                   });
                   
                   let inst_bin = new fabric.Text("", {
                           left: 750,
                           top: 180,
                           fontFamily: "monospace",
                           fill: "black",
                           fontSize: 18
                   });
                   
                   let inst_hex = new fabric.Text("", {
                           left: 750,
                           top: 210,
                           fontFamily: "monospace",
                           fill: "black",
                           fontSize: 18
                   });
                   
                   let inst_header = new fabric.Text("⚙️ Instruction ", {
                        top: 10,
                        left: 830,
                        fontSize: 20,
                        fontWeight: 800,
                        fontFamily: "monospace",
                        fill: "black"
                   })
            
                   return {mem_bg, inst_bg, imem_header, inst_header, inst, inst_type, inst_bin, inst_hex};
               },
               render(){
                  
                  let num = '$instr_rdata[31:0]'.asInt().toString(2);
                  
                  console.log(num)
                  
                  let inst_box = this.getInitObjects().inst;
                  let inst_type = this.getInitObjects().inst_type;
                  let inst_bin = this.getInitObjects().inst_bin
                  let inst_hex = this.getInitObjects().inst_hex
                  
                  if(num !=0){
                     this.instruction.then(function(result) {
                        const inst = new result.Instruction(num, {
                           ABI: false,
                           ISA: isa_cops
                        });
            
                        let inst_str = inst.asm.toString();
                        let type = inst.fmt.toString();
                        let $bin = inst.bin.toString();
                        let $hex = inst.hex.toString();
                        inst_box.set({text: inst_str})
                        inst_type.set({text: type})
                        inst_bin.set({text: "Bin:"+$bin})
                        inst_hex.set({text: "Hex:"+$hex})
                        //this.global.canvas.renderAll.bind(this.global.canvas)()
                     });
                  }
               }
            /all_instrs[31:0]
               $inst[m5_DATA_WIDTH-1:0] = *instrs\[#all_instrs\]; 
               \viz_js
                  layout: "vertical",
                  box: {top: 30, left: 300, strokeWidth: 0, width: 20, height: 25},
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
                     
                     let box = new fabric.Rect({
                           width: 600,
                           height: 20,
                           opacity: 0.1
                     });
                     let asm = new fabric.Text("", {
                           left: 360,
                           fontFamily: "monospace",
                           fill: "black",
                           fontSize: 15,
                     });
                     let bin = new fabric.Text("", {
                           left: 10,
                           fontFamily: "monospace",
                           fill: "black",
                           fontSize: 15,
                     });
                     
                     
                     return {asm, bin, box};
                  },
                  render() {
                     let num = '$inst'.asInt().toString(2);
                     let asm = this.getInitObjects().asm;
                     let box = this.getInitObjects().box;
                     let bin = this.getInitObjects().bin;
               
                     bin.set({text: num});
               
                     if(num != 0){
                        this.instruction.then(function(result) {
                           const inst = new result.Instruction(num, {
                              ABI: false,
                              ISA: isa_cops
                           });
                           let asm_str = inst.asm.toString();
                           asm.set({text: asm_str})
                        });
                     }
                     let curr_instr = '|core/instructions$instr_rdata'.asInt().toString(2)
               
                     if(curr_instr == num){
                        box.set({fill: "gray", width: 600, height: 20, top: 0, left: 0})
                     }else{
                        box.set({fill: "white", width: 600, height: 20, top: 0, left: 0})
                     }
                  },
                  where: {top: 100, left: 300},
                  
         /waterfall
            $if_stage = *if_stage;
            $id_stage = *id_stage;
            $ex_stage = *ex_stage;
            $wb_stage = *wb_stage;
            
            \viz_js
                  box: {strokeWidth: 0},
                  init(){
                     let pipe_img = this.newImageFromURL(
                        "https://raw.githubusercontent.com/chaitravi-ce/makerchip_assets/main/CV32E40P_Pipeline.png", 
                        "",
                        {left: 710, top: 460, width: 450},
                     )
                     
                     let pipe_header = new fabric.Text("Pipeline Reference", {
                           top: 420,
                           left: 830,
                           fontSize: 20,
                           fontWeight: 800,
                           fontFamily: "monospace",
                           fill: "black"
                     })
                     
                     let waterfall_header = new fabric.Text("Waterfall Diagram", {
                           top: 710,
                           left: 830,
                           fontSize: 20,
                           fontWeight: 800,
                           fontFamily: "monospace",
                           fill: "black"
                     })
                     
                     return { pipe_img, pipe_header, waterfall_header }
                  }
            /instrs[4:0]
               \viz_js
                  layout: {
                     left: function(i) {return -i * 30},
                     top: function(i) {return -i * 30},
                  },
                  box: {height: 30, width: 30},
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
                     
                     let instr = new fabric.Text("", {
                           top: 910,
                           left: 830,
                           fontSize: 15,
                           fontFamily: "monospace",
                           fill: "black"
                     })
                     return {instr}
                  },
                  render(){
                     
                     let index = this.getIndex()
                     
                     let rdata = '|core/instructions$instr_rdata'
                     rdata.step(-index)
                     let curr_instr = rdata.asInt().toString(2)
                     let instr = this.getInitObjects().instr;
                     
                     if(curr_instr != 0){
                        this.instruction.then((result) => {
                           const inst = new result.Instruction(curr_instr, {
                              ABI: false,
                              ISA: isa_cops
                           });
                           let asm_str = inst.asm.toString();
                           instr.set({text: asm_str})
                           this.global.canvas.renderAll.bind(this.global.canvas)()
                        });
                     }else{
                        instr.set({text: ""})
                     }
                     
                     let check = -1;
                     let arr = [];
                     for(i=0; i<4; i++){
                        let step = -index+i
                        switch(check){
                           case 0:
                              let id_stage_sig = '|core/waterfall$id_stage';
                              id_stage_sig.step(step);
                              let id_stage_val = id_stage_sig.asInt();
                              if(id_stage_val = 0){
                                 arr.push(0)
                              }else{
                                 check++;
                                 arr.push(1)
                              }
                              break;
                           case 1:
                              let ex_stage_sig = '|core/waterfall$ex_stage';
                              ex_stage_sig.step(step);
                              let ex_stage_val = ex_stage_sig.asInt();
                              if(ex_stage_val = 0){
                                 arr.push(1)
                              }else{
                                 check++;
                                 arr.push(2)
                              }
                              break;
                           case 2:
                              let wb_stage_sig = '|core/waterfall$wb_stage';
                              wb_stage_sig.step(step);
                              let wb_stage_val = wb_stage_sig.asInt();
                              if(wb_stage_val = 0){
                                 arr.push(2)
                              }else{
                                 check++;
                                 arr.push(3)
                              }
                              break;
                           default:
                              let if_stage_sig = '|core/waterfall$if_stage';
                              if_stage_sig.step(step);
                              let if_stage_val = if_stage_sig.asInt();
                              if(if_stage_val == 1){
                                 check++;
                                 arr.push(0)
                              }else{
                                 arr.push(-1)
                              }
                              break;
                        }
                     }
                     let ret_objects = [];
                     
                     for(i=0; i<arr.length; i++){
                        let color = "gray"
                        switch(arr[i]){
                           case -1:
                              color = "gray"
                              break;
                           case 0:
                              color = "darkblue"
                              break;
                           case 1:
                              color = "darkgreen"
                              break;
                           case 2:
                              color = "rgb(114,137,160)"
                              break;
                           case 3:
                              color = "purple"
                              break;
                        }
               
                        ret_objects.push(new fabric.Rect({
                           width: 30,
                           height: 30,
                           opacity: 1,
                           fill: color,
                           left: 1020 + 30*i,
                           top: 900
                        }));
                     }
                     return ret_objects;
                  }
               
         /registers
            /int_reg[15:0]
               $value[m5_DATA_WIDTH-1:0] = *mem\[#int_reg\];
               \viz_js
                  layout: "vertical",
                  box: {top: 0, left: 0, strokeWidth: 5, stroke: "brown", width: 200, height: 30, fill: "white"},
                  render() {
                     let num = '$value'.asInt().toString()
                     let $index = this.getIndex()
                     return [
                        new fabric.Text("x"+$index, {
                              left: 10,
                              fontFamily: "monospace",
                              fill: "black",
                              fontSize: 20,
                        }),
                        new fabric.Text(num, {
                              left: 80,
                              fontFamily: "monospace",
                              fill: "black",
                              fontSize: 20,
                              textAlign: "center"
                      })
                     ]
                  },
                  where: {top: 70, left: 1270},
            
            /fp_reg[15:0]
               $value[m5_DATA_WIDTH-1:0] = *mem_fp\[#fp_reg\]; 
               \viz_js
                  layout: "vertical",
                  box: {top: 0, left: 0, strokeWidth: 5, stroke:"brown", width: 200, height: 30, fill: "white"},
                  render() {
                     let num = '$value'.asInt().toString(16)
                     let $index = this.getIndex()+16
                     return [
                        new fabric.Text("x"+$index, {
                              left: 10,
                              fontFamily: "monospace",
                              fill: "black",
                              fontSize: 20,
                        }),
                        new fabric.Text(num, {
                              left: 80,
                              fontFamily: "monospace",
                              fill: "black",
                              fontSize: 20,
                        })
                     ]
                  },
                  where: {top: 70, left: 1550},
            
            /csr_reg
               $fcsr[31:0] = *fcsr;
               \viz_js
                  box: {fill: "white", height: 20, width: 200},
                  render(){
                     let $fcsr = '$fcsr'.asInt().toString(2)
                     return[
                           new fabric.Text("FCSR:       "+$fcsr, {
                                 left: 10,
                                 fontFamily: "monospace",
                                 fill: "black",
                                 fontSize: 20,
                           })
                     ]
                  },
                  where: {top: 70, left: 1850},
         /mem_word[m5_calc(2**m5_RAM_WIDTH/8-1):0]
            \viz_js
               layout: "vertical",
               box: {strokeWidth: 0},
            /mem_byte[7:0]
               $ram[7:0] = cv32e40p_tb_wrapper_i.ram_i.dp_ram_i.mem[#mem_word * 8 + #mem_byte\];
               \viz_js
                  layout: "horizontal",
                  box: {top: -2, left: -4, strokeWidth: 5, stroke: "green", width: 50, height: 30, fill: "white"},
                  render(){
                     let ram = '$ram'.asInt().toString()
                     return [
                           new fabric.Text(ram, {
                                 fontFamily: "monospace",
                                 fill: "black",
                                 fontSize: 20,
                           })
                       ]
                  },
                  where: {top: 70, left: 2230},
         \viz_js
               box: {strokeWidth: 0},
               init(){
               
                   let regi_header = new fabric.Text("📂 Integer Registers", {
                           top: 10,
                           left: 1250,
                           fontSize: 20,
                           fontWeight: 800,
                           fontFamily: "monospace",
                           fill: "white"
                   })

                   let regf_header = new fabric.Text("📂 FP Registers", {
                           top: 10,
                           left: 1550,
                           fontSize: 20,
                           fontWeight: 800,
                           fontFamily: "monospace",
                           fill: "white"
                   })

                   let regc_header = new fabric.Text("📂 CSR Registers", {
                           top: 10,
                           left: 1850,
                           fontSize: 20,
                           fontWeight: 800,
                           fontFamily: "monospace",
                           fill: "white"
                   })
         
                   let mem_header = new fabric.Text("🗃️ Memory", {
                           top: 10,
                           left: 2350,
                           fontSize: 20,
                           fontWeight: 800,
                           fontFamily: "monospace",
                           fill: "black"
                   })
         
                   let cyc_header = new fabric.Text("Cycle Level Behaviour", {
                           top: 370,
                           left: 780,
                           fontSize: 25,
                           fontWeight: 800,
                           fontFamily: "monospace",
                           fill: "green",
                   })
         
                   let bg = new fabric.Rect({
                              width: 870,
                              height: 600,
                              fill: "brown",
                              left: 1220,
                              top: -10
                   })
         
                   let bg_mem = new fabric.Rect({
                              width: 570,
                              height: 1100,
                              fill: "green",
                              left: 2140,
                              top: -10
                   })
         
                   let bg_cyc = new fabric.Rect({
                              width: 500,
                              height: 660,
                              fill: "green",
                              opacity: 0.4,
                              left: 680,
                              top: 330,
                              stroke: "green",
                              strokeWidth: 6
                   })
         
                  return { bg, bg_mem, bg_cyc, regi_header, regf_header, mem_header, regc_header, cyc_header}
               }
   
   *passed = !clk || *tests_passed_o;
   *failed = !clk || *tests_failed_o || *cyc_cnt > 1000000;
   

\SV
   endmodule
   