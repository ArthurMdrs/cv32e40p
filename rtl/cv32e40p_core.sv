// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Top level module                                           //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Top level module of the RISC-V core.                       //
//                 added APU, FPU parameter to include the APU_dispatcher     //
//                 and the FPU                                                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40p_core
  import cv32e40p_apu_core_pkg::*;
#(
    parameter COREV_PULP =  0,  // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. cv.elw)
    parameter COREV_CLUSTER = 0,  // PULP Cluster interface (incl. cv.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion lanes pipeline registers number
    parameter ZFINX = 0,  // Float-in-General Purpose registers
    parameter NUM_MHPMCOUNTERS = 1
) (
    // Clock and Reset
    input logic clk_i,
    input logic rst_ni,

    input logic pulp_clock_en_i,  // PULP clock enable (only used if COREV_CLUSTER = 1)
    input logic scan_cg_en_i,  // Enable all clock gates for testing

    // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
    input logic [31:0] boot_addr_i,
    input logic [31:0] mtvec_addr_i,
    input logic [31:0] dm_halt_addr_i,
    input logic [31:0] hart_id_i,
    input logic [31:0] dm_exception_addr_i,

    // Instruction memory interface
    output logic        instr_req_o,
    input  logic        instr_gnt_i,
    input  logic        instr_rvalid_i,
    output logic [31:0] instr_addr_o,
    input  logic [31:0] instr_rdata_i,

    // Data memory interface
    output logic        data_req_o,
    input  logic        data_gnt_i,
    input  logic        data_rvalid_i,
    output logic        data_we_o,
    output logic [ 3:0] data_be_o,
    output logic [31:0] data_addr_o,
    output logic [31:0] data_wdata_o,
    input  logic [31:0] data_rdata_i,

    // CVFPU interface
    output logic                              apu_busy_o,
    // handshake signals
    output logic                              apu_req_o,
    input  logic                              apu_gnt_i,
    // request channel
    output logic [   APU_NARGS_CPU-1:0][31:0] apu_operands_o,
    output logic [     APU_WOP_CPU-1:0]       apu_op_o,
    output logic [APU_NDSFLAGS_CPU-1:0]       apu_flags_o,
    // response channel
    input  logic                              apu_rvalid_i,
    input  logic [                31:0]       apu_result_i,
    input  logic [APU_NUSFLAGS_CPU-1:0]       apu_flags_i,

    // Interrupt inputs
    input  logic [31:0] irq_i,  // CLINT interrupts + CLINT extension interrupts
    output logic        irq_ack_o,
    output logic [ 4:0] irq_id_o,

    // Debug Interface
    input  logic debug_req_i,
    output logic debug_havereset_o,
    output logic debug_running_o,
    output logic debug_halted_o,

    // CPU Control Signals
    input  logic fetch_enable_i,
    output logic core_sleep_o

`ifdef RISCV_FORMAL
    ,
    `RVFI_OUTPUTS
`endif
);

  import cv32e40p_pkg::*;

  // Unused parameters and signals (left in code for future design extensions)
  localparam PULP_SECURE = 0;
  localparam N_PMP_ENTRIES = 16;
  localparam USE_PMP = 0;  // if PULP_SECURE is 1, you can still not use the PMP
  localparam A_EXTENSION = 0;
  localparam DEBUG_TRIGGER_EN = 1;

  // PULP bus interface behavior
  // If enabled will allow non-stable address phase signals during waited instructions requests and
  // will re-introduce combinatorial paths from instr_rvalid_i to instr_req_o and from from data_rvalid_i
  // to data_req_o
  localparam PULP_OBI = 0;

  // Unused signals related to above unused parameters
  // Left in code (with their original _i, _o postfixes) for future design extensions;
  // these used to be former inputs/outputs of RI5CY

  logic [5:0] data_atop_o;  // atomic operation, only active if parameter `A_EXTENSION != 0`
  logic       irq_sec_i;
  logic       sec_lvl_o;

  localparam N_HWLP = 2;
  localparam APU = (FPU == 1) ? 1 : 0;

  // IF/ID signals
  logic        instr_valid_id;
  logic [31:0] instr_rdata_id;  // Instruction sampled inside IF stage
  logic        is_compressed_id;
  logic        illegal_c_insn_id;
  logic        is_fetch_failed_id;

  logic        clear_instr_valid;
  logic        pc_set;

  logic [ 3:0] pc_mux_id;  // Mux selector for next PC
  logic [ 2:0] exc_pc_mux_id;  // Mux selector for exception PC
  logic [ 4:0] m_exc_vec_pc_mux_id;  // Mux selector for vectored IRQ PC
  logic [ 4:0] u_exc_vec_pc_mux_id;  // Mux selector for vectored IRQ PC
  logic [ 4:0] exc_cause;

  logic [ 1:0] trap_addr_mux;

  logic [31:0] pc_if;  // Program counter in IF stage
  logic [31:0] pc_id;  // Program counter in ID stage

  // ID performance counter signals
  logic        is_decoding;

  logic        useincr_addr_ex;  // Active when post increment
  logic        data_misaligned;

  logic        mult_multicycle;

  // Jump and branch target and decision (EX->IF)
  logic [31:0] jump_target_id, jump_target_ex;
  logic               branch_in_ex;
  logic               branch_decision;
  logic        [ 1:0] ctrl_transfer_insn_in_dec;

  logic               ctrl_busy;
  logic               if_busy;
  logic               lsu_busy;

  logic        [31:0] pc_ex;  // PC of last executed branch or cv.elw

  // ALU Control
  logic               alu_en_ex;
  alu_opcode_e        alu_operator_ex;
  logic        [31:0] alu_operand_a_ex;
  logic        [31:0] alu_operand_b_ex;
  logic        [31:0] alu_operand_c_ex;
  logic        [ 4:0] bmask_a_ex;
  logic        [ 4:0] bmask_b_ex;
  logic        [ 1:0] imm_vec_ext_ex;
  logic        [ 1:0] alu_vec_mode_ex;
  logic alu_is_clpx_ex, alu_is_subrot_ex;
  logic        [                 1:0]       alu_clpx_shift_ex;

  // Multiplier Control
  mul_opcode_e                              mult_operator_ex;
  logic        [                31:0]       mult_operand_a_ex;
  logic        [                31:0]       mult_operand_b_ex;
  logic        [                31:0]       mult_operand_c_ex;
  logic                                     mult_en_ex;
  logic                                     mult_sel_subword_ex;
  logic        [                 1:0]       mult_signed_mode_ex;
  logic        [                 4:0]       mult_imm_ex;
  logic        [                31:0]       mult_dot_op_a_ex;
  logic        [                31:0]       mult_dot_op_b_ex;
  logic        [                31:0]       mult_dot_op_c_ex;
  logic        [                 1:0]       mult_dot_signed_ex;
  logic                                     mult_is_clpx_ex;
  logic        [                 1:0]       mult_clpx_shift_ex;
  logic                                     mult_clpx_img_ex;

  // FPU
  logic                                     fs_off;
  logic        [            C_RM-1:0]       frm_csr;
  logic        [         C_FFLAG-1:0]       fflags_csr;
  logic                                     fflags_we;
  logic                                     fregs_we;

  // APU
  logic                                     apu_en_ex;
  logic        [APU_NDSFLAGS_CPU-1:0]       apu_flags_ex;
  logic        [     APU_WOP_CPU-1:0]       apu_op_ex;
  logic        [                 1:0]       apu_lat_ex;
  logic        [   APU_NARGS_CPU-1:0][31:0] apu_operands_ex;
  logic        [                 5:0]       apu_waddr_ex;

  logic        [                 2:0][ 5:0] apu_read_regs;
  logic        [                 2:0]       apu_read_regs_valid;
  logic                                     apu_read_dep;
  logic                                     apu_read_dep_for_jalr;
  logic        [                 1:0][ 5:0] apu_write_regs;
  logic        [                 1:0]       apu_write_regs_valid;
  logic                                     apu_write_dep;

  logic                                     perf_apu_type;
  logic                                     perf_apu_cont;
  logic                                     perf_apu_dep;
  logic                                     perf_apu_wb;

  // Register Write Control
  logic        [                 5:0]       regfile_waddr_ex;
  logic                                     regfile_we_ex;
  logic        [                 5:0]       regfile_waddr_fw_wb_o;  // From WB to ID
  logic                                     regfile_we_wb;
  logic                                     regfile_we_wb_power;
  logic        [                31:0]       regfile_wdata;

  logic        [                 5:0]       regfile_alu_waddr_ex;
  logic                                     regfile_alu_we_ex;

  logic        [                 5:0]       regfile_alu_waddr_fw;
  logic                                     regfile_alu_we_fw;
  logic                                     regfile_alu_we_fw_power;
  logic        [                31:0]       regfile_alu_wdata_fw;

  // CSR control
  logic                                     csr_access_ex;
  csr_opcode_e                              csr_op_ex;
  logic [23:0] mtvec, utvec;
  logic        [ 1:0] mtvec_mode;
  logic        [ 1:0] utvec_mode;

  csr_opcode_e        csr_op;
  csr_num_e           csr_addr;
  csr_num_e           csr_addr_int;
  logic        [31:0] csr_rdata;
  logic        [31:0] csr_wdata;
  PrivLvl_t           current_priv_lvl;

  // Data Memory Control:  From ID stage (id-ex pipe) <--> load store unit
  logic               data_we_ex;
  logic        [ 5:0] data_atop_ex;
  logic        [ 1:0] data_type_ex;
  logic        [ 1:0] data_sign_ext_ex;
  logic        [ 1:0] data_reg_offset_ex;
  logic               data_req_ex;
  logic               data_load_event_ex;
  logic               data_misaligned_ex;

  logic               p_elw_start;  // Start of cv.elw load (when data_req_o is sent)
  logic               p_elw_finish;  // Finish of cv.elw load (when data_rvalid_i is received)

  logic        [31:0] lsu_rdata;

  // stall control
  logic               halt_if;
  logic               id_ready;
  logic               ex_ready;

  logic               id_valid;
  logic               ex_valid;
  logic               wb_valid;

  logic               lsu_ready_ex;
  logic               lsu_ready_wb;

  logic               apu_ready_wb;

  // Signals between instruction core interface and pipe (if and id stages)
  logic               instr_req_int;  // Id stage asserts a req to instruction core interface

  // Interrupts
  logic m_irq_enable, u_irq_enable;
  logic csr_irq_sec;
  logic [31:0] mepc, uepc, depc;
  logic [             31:0]       mie_bypass;
  logic [             31:0]       mip;

  logic                           csr_save_cause;
  logic                           csr_save_if;
  logic                           csr_save_id;
  logic                           csr_save_ex;
  logic [              5:0]       csr_cause;
  logic                           csr_restore_mret_id;
  logic                           csr_restore_uret_id;
  logic                           csr_restore_dret_id;
  logic                           csr_mtvec_init;

  // HPM related control signals
  logic [             31:0]       mcounteren;

  // debug mode and dcsr configuration
  logic                           debug_mode;
  logic [              2:0]       debug_cause;
  logic                           debug_csr_save;
  logic                           debug_single_step;
  logic                           debug_ebreakm;
  logic                           debug_ebreaku;
  logic                           trigger_match;
  logic                           debug_p_elw_no_sleep;

  // Hardware loop controller signals
  logic [       N_HWLP-1:0][31:0] hwlp_start;
  logic [       N_HWLP-1:0][31:0] hwlp_end;
  logic [       N_HWLP-1:0][31:0] hwlp_cnt;

  logic [             31:0]       hwlp_target;
  logic                           hwlp_jump;

  // Performance Counters
  logic                           mhpmevent_minstret;
  logic                           mhpmevent_load;
  logic                           mhpmevent_store;
  logic                           mhpmevent_jump;
  logic                           mhpmevent_branch;
  logic                           mhpmevent_branch_taken;
  logic                           mhpmevent_compressed;
  logic                           mhpmevent_jr_stall;
  logic                           mhpmevent_imiss;
  logic                           mhpmevent_ld_stall;
  logic                           mhpmevent_pipe_stall;

  logic                           perf_imiss;

  // Wake signal
  logic                           wake_from_sleep;

  // PMP signals
  logic [N_PMP_ENTRIES-1:0][31:0] pmp_addr;
  logic [N_PMP_ENTRIES-1:0][ 7:0] pmp_cfg;

  logic                           data_req_pmp;
  logic [             31:0]       data_addr_pmp;
  logic                           data_gnt_pmp;
  logic                           data_err_pmp;
  logic                           data_err_ack;
  logic                           instr_req_pmp;
  logic                           instr_gnt_pmp;
  logic [             31:0]       instr_addr_pmp;
  logic                           instr_err_pmp;

  // Mux selector for vectored IRQ PC
  assign m_exc_vec_pc_mux_id = (mtvec_mode == 2'b0) ? 5'h0 : exc_cause;
  assign u_exc_vec_pc_mux_id = (utvec_mode == 2'b0) ? 5'h0 : exc_cause;

  // PULP_SECURE == 0
  assign irq_sec_i = 1'b0;

  // APU master signals
  assign apu_flags_o = apu_flags_ex;

  //////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ _            _      __  __                                                   _    //
  //  / ___| | ___   ___| | __ |  \/  | __ _ _ __   __ _  __ _  ___ _ __ ___   ___ _ __ | |_  //
  // | |   | |/ _ \ / __| |/ / | |\/| |/ _` | '_ \ / _` |/ _` |/ _ \ '_ ` _ \ / _ \ '_ \| __| //
  // | |___| | (_) | (__|   <  | |  | | (_| | | | | (_| | (_| |  __/ | | | | |  __/ | | | |_  //
  //  \____|_|\___/ \___|_|\_\ |_|  |_|\__,_|_| |_|\__,_|\__, |\___|_| |_| |_|\___|_| |_|\__| //
  //                                                     |___/                                //
  //////////////////////////////////////////////////////////////////////////////////////////////

  logic clk;
  logic fetch_enable;

  cv32e40p_sleep_unit #(
      .COREV_CLUSTER(COREV_CLUSTER)
  ) sleep_unit_i (
      // Clock, reset interface
      .clk_ungated_i(clk_i),  // Ungated clock
      .rst_n        (rst_ni),
      .clk_gated_o  (clk),  // Gated clock
      .scan_cg_en_i (scan_cg_en_i),

      // Core sleep
      .core_sleep_o(core_sleep_o),

      // Fetch enable
      .fetch_enable_i(fetch_enable_i),
      .fetch_enable_o(fetch_enable),

      // Core status
      .if_busy_i  (if_busy),
      .ctrl_busy_i(ctrl_busy),
      .lsu_busy_i (lsu_busy),
      .apu_busy_i (apu_busy_o),

      // PULP cluster
      .pulp_clock_en_i       (pulp_clock_en_i),
      .p_elw_start_i         (p_elw_start),
      .p_elw_finish_i        (p_elw_finish),
      .debug_p_elw_no_sleep_i(debug_p_elw_no_sleep),

      // WFI wake
      .wake_from_sleep_i(wake_from_sleep)
  );


  //////////////////////////////////////////////////
  //   ___ _____   ____ _____  _    ____ _____    //
  //  |_ _|  ___| / ___|_   _|/ \  / ___| ____|   //
  //   | || |_    \___ \ | | / _ \| |  _|  _|     //
  //   | ||  _|    ___) || |/ ___ \ |_| | |___    //
  //  |___|_|     |____/ |_/_/   \_\____|_____|   //
  //                                              //
  //////////////////////////////////////////////////
  cv32e40p_if_stage #(
      .COREV_PULP (COREV_PULP),
      .PULP_OBI   (PULP_OBI),
      .PULP_SECURE(PULP_SECURE),
      .FPU        (FPU),
      .ZFINX      (ZFINX)
  ) if_stage_i (
      .clk  (clk),
      .rst_n(rst_ni),

      // boot address
      .boot_addr_i        (boot_addr_i[31:0]),
      .dm_exception_addr_i(dm_exception_addr_i[31:0]),

      // debug mode halt address
      .dm_halt_addr_i(dm_halt_addr_i[31:0]),

      // trap vector location
      .m_trap_base_addr_i(mtvec),
      .u_trap_base_addr_i(utvec),
      .trap_addr_mux_i   (trap_addr_mux),

      // instruction request control
      .req_i(instr_req_int),

      // instruction cache interface
      .instr_req_o    (instr_req_pmp),
      .instr_addr_o   (instr_addr_pmp),
      .instr_gnt_i    (instr_gnt_pmp),
      .instr_rvalid_i (instr_rvalid_i),
      .instr_rdata_i  (instr_rdata_i),
      .instr_err_i    (1'b0),  // Bus error (not used yet)
      .instr_err_pmp_i(instr_err_pmp),  // PMP error

      // outputs to ID stage
      .instr_valid_id_o (instr_valid_id),
      .instr_rdata_id_o (instr_rdata_id),
      .is_fetch_failed_o(is_fetch_failed_id),

      // control signals
      .clear_instr_valid_i(clear_instr_valid),
      .pc_set_i           (pc_set),

      .mepc_i(mepc),  // exception return address
      .uepc_i(uepc),  // exception return address

      .depc_i(depc),  // debug return address

      .pc_mux_i    (pc_mux_id),  // sel for pc multiplexer
      .exc_pc_mux_i(exc_pc_mux_id),


      .pc_id_o(pc_id),
      .pc_if_o(pc_if),

      .is_compressed_id_o (is_compressed_id),
      .illegal_c_insn_id_o(illegal_c_insn_id),

      .m_exc_vec_pc_mux_i(m_exc_vec_pc_mux_id),
      .u_exc_vec_pc_mux_i(u_exc_vec_pc_mux_id),

      .csr_mtvec_init_o(csr_mtvec_init),

      // from hwloop registers
      .hwlp_jump_i  (hwlp_jump),
      .hwlp_target_i(hwlp_target),


      // Jump targets
      .jump_target_id_i(jump_target_id),
      .jump_target_ex_i(jump_target_ex),

      // pipeline stalls
      .halt_if_i (halt_if),
      .id_ready_i(id_ready),

      .if_busy_o   (if_busy),
      .perf_imiss_o(perf_imiss)
  );


  /////////////////////////////////////////////////
  //   ___ ____    ____ _____  _    ____ _____   //
  //  |_ _|  _ \  / ___|_   _|/ \  / ___| ____|  //
  //   | || | | | \___ \ | | / _ \| |  _|  _|    //
  //   | || |_| |  ___) || |/ ___ \ |_| | |___   //
  //  |___|____/  |____/ |_/_/   \_\____|_____|  //
  //                                             //
  /////////////////////////////////////////////////
  cv32e40p_id_stage #(
      .COREV_PULP      (COREV_PULP),
      .COREV_CLUSTER   (COREV_CLUSTER),
      .N_HWLP          (N_HWLP),
      .PULP_SECURE     (PULP_SECURE),
      .USE_PMP         (USE_PMP),
      .A_EXTENSION     (A_EXTENSION),
      .APU             (APU),
      .FPU             (FPU),
      .FPU_ADDMUL_LAT  (FPU_ADDMUL_LAT),
      .FPU_OTHERS_LAT  (FPU_OTHERS_LAT),
      .ZFINX           (ZFINX),
      .APU_NARGS_CPU   (APU_NARGS_CPU),
      .APU_WOP_CPU     (APU_WOP_CPU),
      .APU_NDSFLAGS_CPU(APU_NDSFLAGS_CPU),
      .APU_NUSFLAGS_CPU(APU_NUSFLAGS_CPU),
      .DEBUG_TRIGGER_EN(DEBUG_TRIGGER_EN)
  ) id_stage_i (
      .clk          (clk),  // Gated clock
      .clk_ungated_i(clk_i),  // Ungated clock
      .rst_n        (rst_ni),

      .scan_cg_en_i(scan_cg_en_i),

      // Processor Enable
      .fetch_enable_i               ( fetch_enable         ),     // Delayed version so that clock can remain gated until fetch enabled
      .ctrl_busy_o(ctrl_busy),
      .is_decoding_o(is_decoding),

      // Interface to instruction memory
      .instr_valid_i(instr_valid_id),
      .instr_rdata_i(instr_rdata_id),
      .instr_req_o  (instr_req_int),

      // Jumps and branches
      .branch_in_ex_o             (branch_in_ex),
      .branch_decision_i          (branch_decision),
      .jump_target_o              (jump_target_id),
      .ctrl_transfer_insn_in_dec_o(ctrl_transfer_insn_in_dec),

      // IF and ID control signals
      .clear_instr_valid_o(clear_instr_valid),
      .pc_set_o           (pc_set),
      .pc_mux_o           (pc_mux_id),
      .exc_pc_mux_o       (exc_pc_mux_id),
      .exc_cause_o        (exc_cause),
      .trap_addr_mux_o    (trap_addr_mux),

      .is_fetch_failed_i(is_fetch_failed_id),

      .pc_id_i(pc_id),

      .is_compressed_i (is_compressed_id),
      .illegal_c_insn_i(illegal_c_insn_id),

      // Stalls
      .halt_if_o(halt_if),

      .id_ready_o(id_ready),
      .ex_ready_i(ex_ready),
      .wb_ready_i(lsu_ready_wb),

      .id_valid_o(id_valid),
      .ex_valid_i(ex_valid),

      // From the Pipeline ID/EX
      .pc_ex_o(pc_ex),

      .alu_en_ex_o        (alu_en_ex),
      .alu_operator_ex_o  (alu_operator_ex),
      .alu_operand_a_ex_o (alu_operand_a_ex),
      .alu_operand_b_ex_o (alu_operand_b_ex),
      .alu_operand_c_ex_o (alu_operand_c_ex),
      .bmask_a_ex_o       (bmask_a_ex),
      .bmask_b_ex_o       (bmask_b_ex),
      .imm_vec_ext_ex_o   (imm_vec_ext_ex),
      .alu_vec_mode_ex_o  (alu_vec_mode_ex),
      .alu_is_clpx_ex_o   (alu_is_clpx_ex),
      .alu_is_subrot_ex_o (alu_is_subrot_ex),
      .alu_clpx_shift_ex_o(alu_clpx_shift_ex),

      .regfile_waddr_ex_o(regfile_waddr_ex),
      .regfile_we_ex_o   (regfile_we_ex),

      .regfile_alu_we_ex_o   (regfile_alu_we_ex),
      .regfile_alu_waddr_ex_o(regfile_alu_waddr_ex),

      // MUL
      .mult_operator_ex_o   (mult_operator_ex),  // from ID to EX stage
      .mult_en_ex_o         (mult_en_ex),  // from ID to EX stage
      .mult_sel_subword_ex_o(mult_sel_subword_ex),  // from ID to EX stage
      .mult_signed_mode_ex_o(mult_signed_mode_ex),  // from ID to EX stage
      .mult_operand_a_ex_o  (mult_operand_a_ex),  // from ID to EX stage
      .mult_operand_b_ex_o  (mult_operand_b_ex),  // from ID to EX stage
      .mult_operand_c_ex_o  (mult_operand_c_ex),  // from ID to EX stage
      .mult_imm_ex_o        (mult_imm_ex),  // from ID to EX stage

      .mult_dot_op_a_ex_o  (mult_dot_op_a_ex),  // from ID to EX stage
      .mult_dot_op_b_ex_o  (mult_dot_op_b_ex),  // from ID to EX stage
      .mult_dot_op_c_ex_o  (mult_dot_op_c_ex),  // from ID to EX stage
      .mult_dot_signed_ex_o(mult_dot_signed_ex),  // from ID to EX stage
      .mult_is_clpx_ex_o   (mult_is_clpx_ex),  // from ID to EX stage
      .mult_clpx_shift_ex_o(mult_clpx_shift_ex),  // from ID to EX stage
      .mult_clpx_img_ex_o  (mult_clpx_img_ex),  // from ID to EX stage

      // FPU
      .fs_off_i(fs_off),
      .frm_i   (frm_csr),

      // APU
      .apu_en_ex_o      (apu_en_ex),
      .apu_op_ex_o      (apu_op_ex),
      .apu_lat_ex_o     (apu_lat_ex),
      .apu_operands_ex_o(apu_operands_ex),
      .apu_flags_ex_o   (apu_flags_ex),
      .apu_waddr_ex_o   (apu_waddr_ex),

      .apu_read_regs_o        (apu_read_regs),
      .apu_read_regs_valid_o  (apu_read_regs_valid),
      .apu_read_dep_i         (apu_read_dep),
      .apu_read_dep_for_jalr_i(apu_read_dep_for_jalr),
      .apu_write_regs_o       (apu_write_regs),
      .apu_write_regs_valid_o (apu_write_regs_valid),
      .apu_write_dep_i        (apu_write_dep),
      .apu_perf_dep_o         (perf_apu_dep),
      .apu_busy_i             (apu_busy_o),

      // CSR ID/EX
      .csr_access_ex_o      (csr_access_ex),
      .csr_op_ex_o          (csr_op_ex),
      .current_priv_lvl_i   (current_priv_lvl),
      .csr_irq_sec_o        (csr_irq_sec),
      .csr_cause_o          (csr_cause),
      .csr_save_if_o        (csr_save_if),  // control signal to save pc
      .csr_save_id_o        (csr_save_id),  // control signal to save pc
      .csr_save_ex_o        (csr_save_ex),  // control signal to save pc
      .csr_restore_mret_id_o(csr_restore_mret_id),  // control signal to restore pc
      .csr_restore_uret_id_o(csr_restore_uret_id),  // control signal to restore pc

      .csr_restore_dret_id_o(csr_restore_dret_id),  // control signal to restore pc

      .csr_save_cause_o(csr_save_cause),

      // hardware loop signals to IF hwlp controller
      .hwlp_start_o(hwlp_start),
      .hwlp_end_o  (hwlp_end),
      .hwlp_cnt_o  (hwlp_cnt),

      .hwlp_jump_o  (hwlp_jump),
      .hwlp_target_o(hwlp_target),

      // LSU
      .data_req_ex_o       (data_req_ex),  // to load store unit
      .data_we_ex_o        (data_we_ex),  // to load store unit
      .atop_ex_o           (data_atop_ex),
      .data_type_ex_o      (data_type_ex),  // to load store unit
      .data_sign_ext_ex_o  (data_sign_ext_ex),  // to load store unit
      .data_reg_offset_ex_o(data_reg_offset_ex),  // to load store unit
      .data_load_event_ex_o(data_load_event_ex),  // to load store unit

      .data_misaligned_ex_o(data_misaligned_ex),  // to load store unit

      .prepost_useincr_ex_o(useincr_addr_ex),
      .data_misaligned_i   (data_misaligned),
      .data_err_i          (data_err_pmp),
      .data_err_ack_o      (data_err_ack),

      // Interrupt Signals
      .irq_i         (irq_i),
      .irq_sec_i     ((PULP_SECURE) ? irq_sec_i : 1'b0),
      .mie_bypass_i  (mie_bypass),
      .mip_o         (mip),
      .m_irq_enable_i(m_irq_enable),
      .u_irq_enable_i(u_irq_enable),
      .irq_ack_o     (irq_ack_o),
      .irq_id_o      (irq_id_o),

      // Debug Signal
      .debug_mode_o          (debug_mode),
      .debug_cause_o         (debug_cause),
      .debug_csr_save_o      (debug_csr_save),
      .debug_req_i           (debug_req_i),
      .debug_havereset_o     (debug_havereset_o),
      .debug_running_o       (debug_running_o),
      .debug_halted_o        (debug_halted_o),
      .debug_single_step_i   (debug_single_step),
      .debug_ebreakm_i       (debug_ebreakm),
      .debug_ebreaku_i       (debug_ebreaku),
      .trigger_match_i       (trigger_match),
      .debug_p_elw_no_sleep_o(debug_p_elw_no_sleep),

      // Wakeup Signal
      .wake_from_sleep_o(wake_from_sleep),

      // Forward Signals
      .regfile_waddr_wb_i   (regfile_waddr_fw_wb_o),  // Write address ex-wb pipeline
      .regfile_we_wb_i      (regfile_we_wb),  // write enable for the register file
      .regfile_we_wb_power_i(regfile_we_wb_power),
      .regfile_wdata_wb_i   (regfile_wdata),  // write data to commit in the register file

      .regfile_alu_waddr_fw_i   (regfile_alu_waddr_fw),
      .regfile_alu_we_fw_i      (regfile_alu_we_fw),
      .regfile_alu_we_fw_power_i(regfile_alu_we_fw_power),
      .regfile_alu_wdata_fw_i   (regfile_alu_wdata_fw),

      // from ALU
      .mult_multicycle_i(mult_multicycle),

      // Performance Counters
      .mhpmevent_minstret_o    (mhpmevent_minstret),
      .mhpmevent_load_o        (mhpmevent_load),
      .mhpmevent_store_o       (mhpmevent_store),
      .mhpmevent_jump_o        (mhpmevent_jump),
      .mhpmevent_branch_o      (mhpmevent_branch),
      .mhpmevent_branch_taken_o(mhpmevent_branch_taken),
      .mhpmevent_compressed_o  (mhpmevent_compressed),
      .mhpmevent_jr_stall_o    (mhpmevent_jr_stall),
      .mhpmevent_imiss_o       (mhpmevent_imiss),
      .mhpmevent_ld_stall_o    (mhpmevent_ld_stall),
      .mhpmevent_pipe_stall_o  (mhpmevent_pipe_stall),

      .perf_imiss_i(perf_imiss),
      .mcounteren_i(mcounteren)
  );


  /////////////////////////////////////////////////////
  //   _______  __  ____ _____  _    ____ _____      //
  //  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
  //  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
  //  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
  //  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
  //                                                 //
  /////////////////////////////////////////////////////
  cv32e40p_ex_stage #(
      .COREV_PULP      (COREV_PULP),
      .FPU             (FPU),
      .APU_NARGS_CPU   (APU_NARGS_CPU),
      .APU_WOP_CPU     (APU_WOP_CPU),
      .APU_NDSFLAGS_CPU(APU_NDSFLAGS_CPU),
      .APU_NUSFLAGS_CPU(APU_NUSFLAGS_CPU)
  ) ex_stage_i (
      // Global signals: Clock and active low asynchronous reset
      .clk  (clk),
      .rst_n(rst_ni),

      // Alu signals from ID stage
      .alu_en_i        (alu_en_ex),
      .alu_operator_i  (alu_operator_ex),  // from ID/EX pipe registers
      .alu_operand_a_i (alu_operand_a_ex),  // from ID/EX pipe registers
      .alu_operand_b_i (alu_operand_b_ex),  // from ID/EX pipe registers
      .alu_operand_c_i (alu_operand_c_ex),  // from ID/EX pipe registers
      .bmask_a_i       (bmask_a_ex),  // from ID/EX pipe registers
      .bmask_b_i       (bmask_b_ex),  // from ID/EX pipe registers
      .imm_vec_ext_i   (imm_vec_ext_ex),  // from ID/EX pipe registers
      .alu_vec_mode_i  (alu_vec_mode_ex),  // from ID/EX pipe registers
      .alu_is_clpx_i   (alu_is_clpx_ex),  // from ID/EX pipe registers
      .alu_is_subrot_i (alu_is_subrot_ex),  // from ID/Ex pipe registers
      .alu_clpx_shift_i(alu_clpx_shift_ex),  // from ID/EX pipe registers

      // Multipler
      .mult_operator_i   (mult_operator_ex),  // from ID/EX pipe registers
      .mult_operand_a_i  (mult_operand_a_ex),  // from ID/EX pipe registers
      .mult_operand_b_i  (mult_operand_b_ex),  // from ID/EX pipe registers
      .mult_operand_c_i  (mult_operand_c_ex),  // from ID/EX pipe registers
      .mult_en_i         (mult_en_ex),  // from ID/EX pipe registers
      .mult_sel_subword_i(mult_sel_subword_ex),  // from ID/EX pipe registers
      .mult_signed_mode_i(mult_signed_mode_ex),  // from ID/EX pipe registers
      .mult_imm_i        (mult_imm_ex),  // from ID/EX pipe registers
      .mult_dot_op_a_i   (mult_dot_op_a_ex),  // from ID/EX pipe registers
      .mult_dot_op_b_i   (mult_dot_op_b_ex),  // from ID/EX pipe registers
      .mult_dot_op_c_i   (mult_dot_op_c_ex),  // from ID/EX pipe registers
      .mult_dot_signed_i (mult_dot_signed_ex),  // from ID/EX pipe registers
      .mult_is_clpx_i    (mult_is_clpx_ex),  // from ID/EX pipe registers
      .mult_clpx_shift_i (mult_clpx_shift_ex),  // from ID/EX pipe registers
      .mult_clpx_img_i   (mult_clpx_img_ex),  // from ID/EX pipe registers

      .mult_multicycle_o(mult_multicycle),  // to ID/EX pipe registers

      .data_req_i          (data_req_o),  // from ID/EX pipeline
      .data_rvalid_i       (data_rvalid_i),  // from ID/EX pipeline
      .data_misaligned_ex_i(data_misaligned_ex),  // from ID/EX pipeline
      .data_misaligned_i   (data_misaligned),

      .ctrl_transfer_insn_in_dec_i(ctrl_transfer_insn_in_dec),

      // FPU
      .fpu_fflags_we_o(fflags_we),
      .fpu_fflags_o   (fflags_csr),

      // APU
      .apu_en_i      (apu_en_ex),
      .apu_op_i      (apu_op_ex),
      .apu_lat_i     (apu_lat_ex),
      .apu_operands_i(apu_operands_ex),
      .apu_waddr_i   (apu_waddr_ex),

      .apu_read_regs_i        (apu_read_regs),
      .apu_read_regs_valid_i  (apu_read_regs_valid),
      .apu_read_dep_o         (apu_read_dep),
      .apu_read_dep_for_jalr_o(apu_read_dep_for_jalr),
      .apu_write_regs_i       (apu_write_regs),
      .apu_write_regs_valid_i (apu_write_regs_valid),
      .apu_write_dep_o        (apu_write_dep),

      .apu_perf_type_o(perf_apu_type),
      .apu_perf_cont_o(perf_apu_cont),
      .apu_perf_wb_o  (perf_apu_wb),
      .apu_ready_wb_o (apu_ready_wb),
      .apu_busy_o     (apu_busy_o),

      // CVFPU interface
      // handshake signals
      .apu_req_o     (apu_req_o),
      .apu_gnt_i     (apu_gnt_i),
      // request channel
      .apu_operands_o(apu_operands_o),
      .apu_op_o      (apu_op_o),
      // response channel
      .apu_rvalid_i  (apu_rvalid_i),
      .apu_result_i  (apu_result_i),
      .apu_flags_i   (apu_flags_i),

      .lsu_en_i   (data_req_ex),
      .lsu_rdata_i(lsu_rdata),

      // interface with CSRs
      .csr_access_i(csr_access_ex),
      .csr_rdata_i (csr_rdata),

      // From ID Stage: Regfile control signals
      .branch_in_ex_i     (branch_in_ex),
      .regfile_alu_waddr_i(regfile_alu_waddr_ex),
      .regfile_alu_we_i   (regfile_alu_we_ex),

      .regfile_waddr_i(regfile_waddr_ex),
      .regfile_we_i   (regfile_we_ex),

      // Output of ex stage pipeline
      .regfile_waddr_wb_o   (regfile_waddr_fw_wb_o),
      .regfile_we_wb_o      (regfile_we_wb),
      .regfile_we_wb_power_o(regfile_we_wb_power),
      .regfile_wdata_wb_o   (regfile_wdata),

      // To IF: Jump and branch target and decision
      .jump_target_o    (jump_target_ex),
      .branch_decision_o(branch_decision),

      // To ID stage: Forwarding signals
      .regfile_alu_waddr_fw_o   (regfile_alu_waddr_fw),
      .regfile_alu_we_fw_o      (regfile_alu_we_fw),
      .regfile_alu_we_fw_power_o(regfile_alu_we_fw_power),
      .regfile_alu_wdata_fw_o   (regfile_alu_wdata_fw),

      // stall control
      .is_decoding_i (is_decoding),
      .lsu_ready_ex_i(lsu_ready_ex),
      .lsu_err_i     (data_err_pmp),

      .ex_ready_o(ex_ready),
      .ex_valid_o(ex_valid),
      .wb_ready_i(lsu_ready_wb)
  );


  ////////////////////////////////////////////////////////////////////////////////////////
  //    _     ___    _    ____    ____ _____ ___  ____  _____   _   _ _   _ ___ _____   //
  //   | |   / _ \  / \  |  _ \  / ___|_   _/ _ \|  _ \| ____| | | | | \ | |_ _|_   _|  //
  //   | |  | | | |/ _ \ | | | | \___ \ | || | | | |_) |  _|   | | | |  \| || |  | |    //
  //   | |__| |_| / ___ \| |_| |  ___) || || |_| |  _ <| |___  | |_| | |\  || |  | |    //
  //   |_____\___/_/   \_\____/  |____/ |_| \___/|_| \_\_____|  \___/|_| \_|___| |_|    //
  //                                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////

  cv32e40p_load_store_unit #(
      .PULP_OBI(PULP_OBI)
  ) load_store_unit_i (
      .clk  (clk),
      .rst_n(rst_ni),

      //output to data memory
      .data_req_o    (data_req_pmp),
      .data_gnt_i    (data_gnt_pmp),
      .data_rvalid_i (data_rvalid_i),
      .data_err_i    (1'b0),  // Bus error (not used yet)
      .data_err_pmp_i(data_err_pmp),  // PMP error

      .data_addr_o (data_addr_pmp),
      .data_we_o   (data_we_o),
      .data_atop_o (data_atop_o),
      .data_be_o   (data_be_o),
      .data_wdata_o(data_wdata_o),
      .data_rdata_i(data_rdata_i),

      // signal from ex stage
      .data_we_ex_i        (data_we_ex),
      .data_atop_ex_i      (data_atop_ex),
      .data_type_ex_i      (data_type_ex),
      .data_wdata_ex_i     (alu_operand_c_ex),
      .data_reg_offset_ex_i(data_reg_offset_ex),
      .data_load_event_ex_i(data_load_event_ex),
      .data_sign_ext_ex_i  (data_sign_ext_ex),  // sign extension

      .data_rdata_ex_o  (lsu_rdata),
      .data_req_ex_i    (data_req_ex),
      .operand_a_ex_i   (alu_operand_a_ex),
      .operand_b_ex_i   (alu_operand_b_ex),
      .addr_useincr_ex_i(useincr_addr_ex),

      .data_misaligned_ex_i(data_misaligned_ex),  // from ID/EX pipeline
      .data_misaligned_o   (data_misaligned),

      .p_elw_start_o (p_elw_start),
      .p_elw_finish_o(p_elw_finish),

      // control signals
      .lsu_ready_ex_o(lsu_ready_ex),
      .lsu_ready_wb_o(lsu_ready_wb),

      .busy_o(lsu_busy)
  );

  // Tracer signal
  assign wb_valid = lsu_ready_wb;


  //////////////////////////////////////
  //        ____ ____  ____           //
  //       / ___/ ___||  _ \ ___      //
  //      | |   \___ \| |_) / __|     //
  //      | |___ ___) |  _ <\__ \     //
  //       \____|____/|_| \_\___/     //
  //                                  //
  //   Control and Status Registers   //
  //////////////////////////////////////

  cv32e40p_cs_registers #(
      .N_HWLP          (N_HWLP),
      .A_EXTENSION     (A_EXTENSION),
      .FPU             (FPU),
      .ZFINX           (ZFINX),
      .APU             (APU),
      .PULP_SECURE     (PULP_SECURE),
      .USE_PMP         (USE_PMP),
      .N_PMP_ENTRIES   (N_PMP_ENTRIES),
      .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
      .COREV_PULP      (COREV_PULP),
      .COREV_CLUSTER   (COREV_CLUSTER),
      .DEBUG_TRIGGER_EN(DEBUG_TRIGGER_EN)
  ) cs_registers_i (
      .clk  (clk),
      .rst_n(rst_ni),

      // Hart ID from outside
      .hart_id_i       (hart_id_i),
      .mtvec_o         (mtvec),
      .utvec_o         (utvec),
      .mtvec_mode_o    (mtvec_mode),
      .utvec_mode_o    (utvec_mode),
      // mtvec address
      .mtvec_addr_i    (mtvec_addr_i[31:0]),
      .csr_mtvec_init_i(csr_mtvec_init),
      // Interface to CSRs (SRAM like)
      .csr_addr_i      (csr_addr),
      .csr_wdata_i     (csr_wdata),
      .csr_op_i        (csr_op),
      .csr_rdata_o     (csr_rdata),

      .fs_off_o   (fs_off),
      .frm_o      (frm_csr),
      .fflags_i   (fflags_csr),
      .fflags_we_i(fflags_we),
      .fregs_we_i (fregs_we),

      // Interrupt related control signals
      .mie_bypass_o  (mie_bypass),
      .mip_i         (mip),
      .m_irq_enable_o(m_irq_enable),
      .u_irq_enable_o(u_irq_enable),
      .csr_irq_sec_i (csr_irq_sec),
      .sec_lvl_o     (sec_lvl_o),
      .mepc_o        (mepc),
      .uepc_o        (uepc),

      // HPM related control signals
      .mcounteren_o(mcounteren),

      // debug
      .debug_mode_i       (debug_mode),
      .debug_cause_i      (debug_cause),
      .debug_csr_save_i   (debug_csr_save),
      .depc_o             (depc),
      .debug_single_step_o(debug_single_step),
      .debug_ebreakm_o    (debug_ebreakm),
      .debug_ebreaku_o    (debug_ebreaku),
      .trigger_match_o    (trigger_match),

      .priv_lvl_o(current_priv_lvl),

      .pmp_addr_o(pmp_addr),
      .pmp_cfg_o (pmp_cfg),

      .pc_if_i(pc_if),
      .pc_id_i(pc_id),
      .pc_ex_i(pc_ex),

      .csr_save_if_i     (csr_save_if),
      .csr_save_id_i     (csr_save_id),
      .csr_save_ex_i     (csr_save_ex),
      .csr_restore_mret_i(csr_restore_mret_id),
      .csr_restore_uret_i(csr_restore_uret_id),

      .csr_restore_dret_i(csr_restore_dret_id),

      .csr_cause_i     (csr_cause),
      .csr_save_cause_i(csr_save_cause),

      // from hwloop registers
      .hwlp_start_i(hwlp_start),
      .hwlp_end_i  (hwlp_end),
      .hwlp_cnt_i  (hwlp_cnt),

      // performance counter related signals
      .mhpmevent_minstret_i    (mhpmevent_minstret),
      .mhpmevent_load_i        (mhpmevent_load),
      .mhpmevent_store_i       (mhpmevent_store),
      .mhpmevent_jump_i        (mhpmevent_jump),
      .mhpmevent_branch_i      (mhpmevent_branch),
      .mhpmevent_branch_taken_i(mhpmevent_branch_taken),
      .mhpmevent_compressed_i  (mhpmevent_compressed),
      .mhpmevent_jr_stall_i    (mhpmevent_jr_stall),
      .mhpmevent_imiss_i       (mhpmevent_imiss),
      .mhpmevent_ld_stall_i    (mhpmevent_ld_stall),
      .mhpmevent_pipe_stall_i  (mhpmevent_pipe_stall),
      .apu_typeconflict_i      (perf_apu_type),
      .apu_contention_i        (perf_apu_cont),
      .apu_dep_i               (perf_apu_dep),
      .apu_wb_i                (perf_apu_wb)
  );

  //  CSR access
  assign csr_addr = csr_addr_int;
  assign csr_wdata = alu_operand_a_ex;
  assign csr_op = csr_op_ex;

  assign csr_addr_int = csr_num_e'(csr_access_ex ? alu_operand_b_ex[11:0] : '0);

  //  Floating-Point registers write
  assign fregs_we     = (FPU == 1 & ZFINX == 0) ? ((regfile_alu_we_fw && regfile_alu_waddr_fw[5]) ||
                                                   (regfile_we_wb     && regfile_waddr_fw_wb_o[5]))
                                                : 1'b0;

  ///////////////////////////
  //   ____  __  __ ____   //
  //  |  _ \|  \/  |  _ \  //
  //  | |_) | |\/| | |_) | //
  //  |  __/| |  | |  __/  //
  //  |_|   |_|  |_|_|     //
  //                       //
  ///////////////////////////

  generate
    if (PULP_SECURE && USE_PMP) begin : gen_pmp
      cv32e40p_pmp #(
          .N_PMP_ENTRIES(N_PMP_ENTRIES)
      ) pmp_unit_i (
          .clk  (clk),
          .rst_n(rst_ni),

          .pmp_privil_mode_i(current_priv_lvl),

          .pmp_addr_i(pmp_addr),
          .pmp_cfg_i (pmp_cfg),


          .data_req_i (data_req_pmp),
          .data_addr_i(data_addr_pmp),
          .data_we_i  (data_we_o),
          .data_gnt_o (data_gnt_pmp),

          .data_req_o    (data_req_o),
          .data_gnt_i    (data_gnt_i),
          .data_addr_o   (data_addr_o),
          .data_err_o    (data_err_pmp),
          .data_err_ack_i(data_err_ack),

          .instr_req_i (instr_req_pmp),
          .instr_addr_i(instr_addr_pmp),
          .instr_gnt_o (instr_gnt_pmp),

          .instr_req_o (instr_req_o),
          .instr_gnt_i (instr_gnt_i),
          .instr_addr_o(instr_addr_o),
          .instr_err_o (instr_err_pmp)
      );
    end else begin : gen_no_pmp
      assign instr_req_o   = instr_req_pmp;
      assign instr_addr_o  = instr_addr_pmp;
      assign instr_gnt_pmp = instr_gnt_i;
      assign instr_err_pmp = 1'b0;

      assign data_req_o    = data_req_pmp;
      assign data_addr_o   = data_addr_pmp;
      assign data_gnt_pmp  = data_gnt_i;
      assign data_err_pmp  = 1'b0;
    end
  endgenerate

`ifdef CV32E40P_ASSERT_ON

  //----------------------------------------------------------------------------
  // Assumptions
  //----------------------------------------------------------------------------

  generate
    if (COREV_CLUSTER) begin : gen_pulp_cluster_assumptions

      // Assumptions/requirements on the environment when pulp_clock_en_i = 0
      property p_env_req_0;
        @(posedge clk_i) disable iff (!rst_ni) (pulp_clock_en_i == 1'b0) |-> (irq_i == 'b0) && (debug_req_i == 1'b0) &&
                                                                            (instr_rvalid_i == 1'b0) && (instr_gnt_i == 1'b0) &&
                                                                            (data_rvalid_i == 1'b0) && (data_gnt_i == 1'b0);
      endproperty

      a_env_req_0 :
      assume property (p_env_req_0);

      // Assumptions/requirements on the environment when core_sleep_o = 0
      property p_env_req_1;
        @(posedge clk_i) disable iff (!rst_ni) (core_sleep_o == 1'b0) |-> (pulp_clock_en_i == 1'b1);
      endproperty

      a_env_req_1 :
      assume property (p_env_req_1);

    end
  endgenerate

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  generate
    if (!COREV_CLUSTER) begin : gen_no_pulp_cluster_assertions
      // Check that a taken IRQ is actually enabled (e.g. that we do not react to an IRQ that was just disabled in MIE)
      property p_irq_enabled_0;
        @(posedge clk) disable iff (!rst_ni) (pc_set && (pc_mux_id == PC_EXCEPTION) && (exc_pc_mux_id == EXC_PC_IRQ)) |->
         (cs_registers_i.mie_n[exc_cause] && cs_registers_i.mstatus_q.mie);
      endproperty

      a_irq_enabled_0 :
      assert property (p_irq_enabled_0);

      // Check that a taken IRQ was for an enabled cause and that mstatus.mie gets disabled
      property p_irq_enabled_1;
        @(posedge clk) disable iff (!rst_ni) (pc_set && (pc_mux_id == PC_EXCEPTION) && (exc_pc_mux_id == EXC_PC_IRQ)) |=>
         (cs_registers_i.mcause_q[5] && cs_registers_i.mie_q[cs_registers_i.mcause_q[4:0]] && !cs_registers_i.mstatus_q.mie);
      endproperty

      a_irq_enabled_1 :
      assert property (p_irq_enabled_1);
    end
  endgenerate

  generate
    if (!COREV_PULP) begin : gen_no_pulp_xpulp_assertions

      // Illegal, ECALL, EBRK checks excluded for PULP due to other definition for for Hardware Loop

      // First illegal instruction decoded
      logic        first_illegal_found;
      logic        first_ecall_found;
      logic        first_ebrk_found;
      logic [31:0] expected_illegal_mepc;
      logic [31:0] expected_ecall_mepc;
      logic [31:0] expected_ebrk_mepc;

      always_ff @(posedge clk, negedge rst_ni) begin
        if (rst_ni == 1'b0) begin
          first_illegal_found   <= 1'b0;
          first_ecall_found     <= 1'b0;
          first_ebrk_found      <= 1'b0;
          expected_illegal_mepc <= 32'b0;
          expected_ecall_mepc   <= 32'b0;
          expected_ebrk_mepc    <= 32'b0;
        end else begin
          if (!first_illegal_found && is_decoding && id_valid && id_stage_i.illegal_insn_dec && !id_stage_i.controller_i.debug_mode_n) begin
            first_illegal_found   <= 1'b1;
            expected_illegal_mepc <= pc_id;
          end
          if (!first_ecall_found && is_decoding && id_valid && id_stage_i.ecall_insn_dec && !id_stage_i.controller_i.debug_mode_n) begin
            first_ecall_found   <= 1'b1;
            expected_ecall_mepc <= pc_id;
          end
          if (!first_ebrk_found && is_decoding && id_valid && id_stage_i.ebrk_insn_dec && (id_stage_i.controller_i.ctrl_fsm_ns != DBG_FLUSH)) begin
            first_ebrk_found   <= 1'b1;
            expected_ebrk_mepc <= pc_id;
          end
        end
      end

      // First mepc write for illegal instruction exception
      logic        first_cause_illegal_found;
      logic        first_cause_ecall_found;
      logic        first_cause_ebrk_found;
      logic [31:0] actual_illegal_mepc;
      logic [31:0] actual_ecall_mepc;
      logic [31:0] actual_ebrk_mepc;

      always_ff @(posedge clk, negedge rst_ni) begin
        if (rst_ni == 1'b0) begin
          first_cause_illegal_found <= 1'b0;
          first_cause_ecall_found   <= 1'b0;
          first_cause_ebrk_found    <= 1'b0;
          actual_illegal_mepc       <= 32'b0;
          actual_ecall_mepc         <= 32'b0;
          actual_ebrk_mepc          <= 32'b0;
        end else begin
          if (!first_cause_illegal_found && (cs_registers_i.csr_cause_i == {
                1'b0, EXC_CAUSE_ILLEGAL_INSN
              }) && csr_save_cause) begin
            first_cause_illegal_found <= 1'b1;
            actual_illegal_mepc       <= cs_registers_i.mepc_n;
          end
          if (!first_cause_ecall_found && (cs_registers_i.csr_cause_i == {
                1'b0, EXC_CAUSE_ECALL_MMODE
              }) && csr_save_cause) begin
            first_cause_ecall_found <= 1'b1;
            actual_ecall_mepc       <= cs_registers_i.mepc_n;
          end
          if (!first_cause_ebrk_found && (cs_registers_i.csr_cause_i == {
                1'b0, EXC_CAUSE_BREAKPOINT
              }) && csr_save_cause) begin
            first_cause_ebrk_found <= 1'b1;
            actual_ebrk_mepc       <= cs_registers_i.mepc_n;
          end
        end
      end

      // Check that mepc is updated with PC of illegal instruction
      property p_illegal_mepc;
        @(posedge clk) disable iff (!rst_ni) (first_illegal_found && first_cause_illegal_found) |=> (expected_illegal_mepc == actual_illegal_mepc);
      endproperty

      a_illegal_mepc :
      assert property (p_illegal_mepc);

      // Check that mepc is updated with PC of the ECALL instruction
      property p_ecall_mepc;
        @(posedge clk) disable iff (!rst_ni) (first_ecall_found && first_cause_ecall_found) |=> (expected_ecall_mepc == actual_ecall_mepc);
      endproperty

      a_ecall_mepc :
      assert property (p_ecall_mepc);

      // Check that mepc is updated with PC of EBRK instruction
      property p_ebrk_mepc;
        @(posedge clk) disable iff (!rst_ni) (first_ebrk_found && first_cause_ebrk_found) |=> (expected_ebrk_mepc == actual_ebrk_mepc);
      endproperty

      a_ebrk_mepc :
      assert property (p_ebrk_mepc);

    end
  endgenerate

  // Single Step only decodes one instruction in non debug mode and next instruction decode is in debug mode
  logic inst_taken;
  assign inst_taken = id_valid && is_decoding;

  a_single_step :
  assert property
  (
    @(posedge clk) disable iff (!rst_ni)
    (inst_taken && debug_single_step && ~debug_mode)
    ##1 inst_taken [->1]
    |-> (debug_mode && debug_single_step));

`endif


`ifdef RISCV_FORMAL

    //====================   Instruction Metadata   ====================//
    reg        rvfi_valid_if, rvfi_valid_id, rvfi_valid_ex, rvfi_valid_wb;
    reg [63:0]                                              rvfi_order_wb;
    reg [31:0] rvfi_insn_if , rvfi_insn_id , rvfi_insn_ex , rvfi_insn_wb ;
    reg                       rvfi_trap_id , rvfi_trap_ex , rvfi_trap_wb ;
    reg        rvfi_halt_if , rvfi_halt_id , rvfi_halt_ex , rvfi_halt_wb ;
    reg        rvfi_intr_if , rvfi_intr_id , rvfi_intr_ex , rvfi_intr_wb ;
    reg [ 1:0]                rvfi_mode_id , rvfi_mode_ex , rvfi_mode_wb ;
    
    reg [31:0] rvfi_insn_id_q;
    
    localparam COMP_LW = 5'b00_010, COMP_LWSP = 5'b10_010, COMP_SW = 5'b00_110, COMP_SWSP = 5'b10_110;
    
    wire insn_id_is_div = rvfi_insn_id[ 6: 0] == OPCODE_OP 
                       && rvfi_insn_id[31:25] == 7'h01
                       && rvfi_insn_id[14:12] inside {3'h4, 3'h5, 3'h6, 3'h7}; // div, divu, rem, remu
                       
    wire insn_id_is_mulh = rvfi_insn_id[ 6: 0] == OPCODE_OP 
                        && rvfi_insn_id[31:25] == 7'h01
                        && rvfi_insn_id[14:12] inside {3'h1, 3'h2, 3'h3}; // mulh, mulhu, mulhsu
                        
    wire insn_ex_is_c_load   = {rvfi_insn_ex[1:0], rvfi_insn_ex[15:13]} inside {COMP_LW, COMP_LWSP}; // Compressed Load
    wire insn_ex_is_pri_load = rvfi_insn_ex[6:0] == OPCODE_CUSTOM_0 
                            && rvfi_insn_ex[14:12] inside {3'h0, 3'h4, 3'h1, 3'h5, 3'h2}; // Post-Increment Register-Immediate Load
    wire insn_ex_is_prr_load = rvfi_insn_ex[6:0] == OPCODE_CUSTOM_1 
                            && rvfi_insn_ex[31:25] inside {7'h0, 7'h8, 7'h1, 7'h9, 7'h2} 
                            && rvfi_insn_ex[14:12] == 3'b011; // Post-Increment Register-Register Load
    wire insn_ex_is_rr_load  = rvfi_insn_ex[6:0] == OPCODE_CUSTOM_1 
                            && rvfi_insn_ex[31:25] inside {7'h4, 7'hC, 7'h5, 7'hD, 7'h6} 
                            && rvfi_insn_ex[14:12] == 3'b011; // Register-Register Load
    wire insn_ex_is_p_load   = insn_ex_is_pri_load || insn_ex_is_prr_load || insn_ex_is_rr_load; //Post-Increment Load
    wire insn_ex_is_load     = rvfi_insn_ex[6:0] inside {OPCODE_LOAD, OPCODE_LOAD_FP}
                            || insn_ex_is_c_load
                            || insn_ex_is_p_load;
                        
    wire insn_ex_is_c_store   = {rvfi_insn_ex[1:0], rvfi_insn_ex[15:13]} inside {COMP_SW, COMP_SWSP}; // Compressed Store
    wire insn_ex_is_pri_store = rvfi_insn_ex[6:0] == OPCODE_CUSTOM_1 
                             && rvfi_insn_ex[14:12] inside {3'h0, 3'h1, 3'h2}; // Post-Increment Register-Immediate Store
    wire insn_ex_is_prr_store = rvfi_insn_ex[6:0] == OPCODE_CUSTOM_1 
                             && rvfi_insn_ex[31:25] inside {7'h10, 7'h11, 7'h12} 
                             && rvfi_insn_ex[14:12] == 3'b011; // Post-Increment Register-Register Store
    wire insn_ex_is_rr_store = rvfi_insn_ex[6:0] == OPCODE_CUSTOM_1 
                            && rvfi_insn_ex[31:25] inside {7'h14, 7'h15, 7'h16} 
                            && rvfi_insn_ex[14:12] == 3'b011; // Register-Register Store
    wire insn_ex_is_p_store  = insn_ex_is_pri_store || insn_ex_is_prr_store || insn_ex_is_rr_store; //Post-Increment Store
    wire insn_ex_is_store    = rvfi_insn_ex[6:0] inside {OPCODE_STORE, OPCODE_STORE_FP}
                            || insn_ex_is_c_store
                            || insn_ex_is_p_store;
    
    wire insn_ex_is_mem = insn_ex_is_load || insn_ex_is_store;
    
    wire insn_id_is_csr = rvfi_insn_id[ 6: 0] == OPCODE_SYSTEM 
                       && rvfi_insn_id[14:12] != 3'b000; // Read/modify CSR
    wire insn_ex_is_csr = rvfi_insn_ex[ 6: 0] == OPCODE_SYSTEM 
                       && rvfi_insn_ex[14:12] != 3'b000; // Read/modify CSR
    
    wire stall_ex = load_store_unit_i.cnt_q != 0 && !load_store_unit_i.count_down;
    // wire stall_ex = 1'b0;
    
    logic        rvfi_valid_mask;
    always @(posedge clk or negedge rst_ni)
        if (!rst_ni)
            rvfi_valid_mask <= 1'b0;
        else
        if (load_store_unit_i.lsu_ready_wb_o) begin
            if ((rvfi_pc_rdata_wb == rvfi_pc_rdata_ex) && (rvfi_insn_wb == rvfi_insn_ex)) begin // && !rvfi_is_hwlp_wb
                if (rvfi_valid_wb)
                    rvfi_valid_mask <= 1'b1;
            end
            else begin
                rvfi_valid_mask <= 1'b0;
            end
        end
    
    
    logic misaligned_access;
    logic data_access_error;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_valid_if <= '0;
            rvfi_valid_id <= '0;
            rvfi_valid_ex <= '0;
            rvfi_valid_wb <= '0;
        end
        else begin
            if (if_stage_i.if_ready)
                rvfi_valid_if <= if_stage_i.if_valid && if_stage_i.instr_valid;
                
                
            if (id_stage_i.id_ready_o)
                // Don't assert valid if ID stage is not decoding
                if (!id_stage_i.is_decoding_o)
                    rvfi_valid_id <= 1'b0;
                else
                    rvfi_valid_id <= rvfi_valid_if && id_stage_i.id_valid_o;
                
                
            if (ex_stage_i.ex_ready_o) begin
                // Assert valid when div/rem instr finishes
                // if (insn_id_is_div && ex_stage_i.alu_i.int_div.div_i.State_SP == 2'b10) // 2'b10 = FINISH
                //     rvfi_valid_ex <= 1'b1;
                // else
                // Assert valid when mulh instr finishes (MAYBE REVIEW HOW THIS IS DONE)
                if (insn_id_is_mulh && ex_stage_i.mult_i.mulh_CS == ex_stage_i.mult_i.mulh_CS.last()) // mulh_CS = FINISH
                    rvfi_valid_ex <= 1'b1;
                else
                // Assert valid when a misaligned store completes
                if (insn_ex_is_store && misaligned_access && !load_store_unit_i.data_misaligned_o) // Misaligned store completed 
                    rvfi_valid_ex <= 1'b1;
                else
                    rvfi_valid_ex <= rvfi_valid_id && ex_stage_i.ex_valid_o;
                // Save status of misaligned access and data error for WB stage
                misaligned_access <= load_store_unit_i.data_misaligned_o;
                data_access_error <= load_store_unit_i.data_err_i;
            end
            else
                // rvfi_valid_ex <= 1'b0;
                rvfi_valid_ex <= ex_stage_i.ex_valid_o;
                
            
            // Memory access instructions get special treatment
            if (insn_ex_is_mem)
                // De-assert valid if we're waiting for a misaligned access to complete
                if (misaligned_access)
                    rvfi_valid_wb <= 1'b0;
                // Assert valid when a mem instruction completes (except if there is a data error)
                else if (load_store_unit_i.cnt_q != 0 && load_store_unit_i.data_rvalid_i) // ctn_q counts the transactions
                    rvfi_valid_wb <= (data_access_error) ? 1'b0 : 1'b1;
                else
                    // rvfi_valid_wb <= 1'b0;
                    rvfi_valid_wb <= rvfi_trap_ex;
            else
            if (load_store_unit_i.lsu_ready_wb_o) 
                rvfi_valid_wb <= (rvfi_valid_ex && wb_valid) || rvfi_trap_ex;
            else
                rvfi_valid_wb <= 1'b0; 
        end
    end
    assign rvfi_valid = rvfi_valid_wb && !rvfi_valid_mask;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_order_wb <= '0;
        end
        else begin
            rvfi_order_wb <= rvfi_order_wb + rvfi_valid;
        end
    end
    assign rvfi_order = rvfi_order_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_insn_if <= '0;
            rvfi_insn_id <= '0;
            rvfi_insn_ex <= '0;
            rvfi_insn_wb <= '0;
            rvfi_insn_id_q <= '0;
        end
        else begin
            rvfi_insn_id_q <= rvfi_insn_id;
            // if (if_stage_i.if_ready)
            if (if_stage_i.if_ready && if_stage_i.if_valid && if_stage_i.instr_valid)
                rvfi_insn_if <= (if_stage_i.instr_compressed_int) ? {16'b0, if_stage_i.instr_aligned[15:0]} : if_stage_i.instr_decompressed;
            if (id_stage_i.id_ready_o)
                rvfi_insn_id <= rvfi_insn_if;
            if (ex_stage_i.ex_ready_o)
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                    rvfi_insn_ex <= rvfi_insn_id;
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_insn_wb <= rvfi_insn_ex;
        end
    end
    assign rvfi_insn = rvfi_insn_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_trap_id <= '0;
            rvfi_trap_ex <= '0;
            rvfi_trap_wb <= '0;
        end
        else begin
            if (id_stage_i.id_ready_o)
                // rvfi_trap_id <= id_stage_i.illegal_insn_dec;
                rvfi_trap_id <= id_stage_i.illegal_insn_dec && id_stage_i.is_decoding_o;
            if (ex_stage_i.ex_ready_o)
                rvfi_trap_ex <= rvfi_trap_id;
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_trap_wb <= rvfi_trap_ex;
        end
    end
    assign rvfi_trap = rvfi_trap_wb;
    // rvfi_trap must be set for an instruction that cannot be decoded as a legal instruction, such as 0x00000000.
    // In addition, rvfi_trap must be set for a misaligned memory read or write in PMAs that don't allow 
    // misaligned access, or other memory access violations.
    // rvfi_trap must also be set for a jump instruction that jumps to a misaligned instruction.
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_halt_if <= '0;
            rvfi_halt_id <= '0;
            rvfi_halt_ex <= '0;
            rvfi_halt_wb <= '0;
        end
        else begin
            rvfi_halt_if <= '0;
            rvfi_halt_id <= rvfi_halt_if;
            rvfi_halt_ex <= rvfi_halt_id;
            rvfi_halt_wb <= rvfi_halt_ex;
        end
    end
    // assign rvfi_halt = rvfi_halt_wb;
    assign rvfi_halt = '0; // Todo!!! Make this work with actual halts!!
    
    
    logic next_is_intr;
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            next_is_intr <= '0;
            rvfi_intr_id <= '0;
            rvfi_intr_if <= '0;
            rvfi_intr_ex <= '0;
            rvfi_intr_wb <= '0;
        end
        else begin
            next_is_intr <= if_stage_i.pc_mux_i inside {PC_EXCEPTION};
            // rvfi_intr_if <= next_is_intr;
            // rvfi_intr_id <= rvfi_intr_if;
            // rvfi_intr_ex <= rvfi_intr_id;
            // rvfi_intr_wb <= rvfi_intr_ex;
            
            
            if (if_stage_i.if_ready)
            // if (if_stage_i.if_ready && if_stage_i.if_valid && if_stage_i.instr_valid)
                // rvfi_intr_if <= next_is_intr;
                rvfi_intr_if <= next_is_intr || id_stage_i.mret_insn_dec;
            if (id_stage_i.id_ready_o)
                rvfi_intr_id <= rvfi_intr_if;
            if (ex_stage_i.ex_ready_o)
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                    rvfi_intr_ex <= rvfi_intr_id;
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_intr_wb <= rvfi_intr_ex;
        end
    end
    assign rvfi_intr = rvfi_intr_wb;
    // rvfi_intr must be set for the first instruction that is part of a trap handler,
    // i.e. an instruction that has a rvfi_pc_rdata that does not match the rvfi_pc_wdata
    // of the previous instruction.
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_mode_id <= '0;
            rvfi_mode_ex <= '0;
            rvfi_mode_wb <= '0;
        end
        else begin
            if (id_stage_i.id_ready_o)
                rvfi_mode_id <= id_stage_i.current_priv_lvl_i;
            if (ex_stage_i.ex_ready_o)
                rvfi_mode_ex <= rvfi_mode_id;
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_mode_wb <= rvfi_mode_ex;
        end
    end
    assign rvfi_mode = rvfi_mode_wb; // 0=U-Mode, 1=S-Mode, 2=Reserved, 3=M-Mode    
    // current_priv_lvl is of the following type:
    // // Privileged mode
    // typedef enum logic[1:0] {
    // PRIV_LVL_M = 2'b11,
    // PRIV_LVL_H = 2'b10,
    // PRIV_LVL_S = 2'b01,
    // PRIV_LVL_U = 2'b00
    // } PrivLvl_t;
    
    assign rvfi_ixl = cs_registers_i.MXL;
    // rvfi_ixl must be set to the value of MXL/SXL/UXL in the current privilege level, 
    // using the following encoding: 1=32, 2=64
    
    

    //====================   Integer Register Read/Write   ====================//
    reg [ 4:0] rvfi_rs1_addr_id , rvfi_rs1_addr_ex , rvfi_rs1_addr_wb ;
    reg [ 4:0] rvfi_rs2_addr_id , rvfi_rs2_addr_ex , rvfi_rs2_addr_wb ;
    reg [31:0] rvfi_rs1_rdata_id, rvfi_rs1_rdata_ex, rvfi_rs1_rdata_wb;
    reg [31:0] rvfi_rs2_rdata_id, rvfi_rs2_rdata_ex, rvfi_rs2_rdata_wb;
    reg [ 4:0]                    rvfi_rd_addr_ex  , rvfi_rd_addr_wb  ;
    reg [31:0]                    rvfi_rd_wdata_ex , rvfi_rd_wdata_wb ;
`ifdef RISCV_FORMAL_CUSTOM_ISA
    reg [ 4:0] rvfi_rs3_addr_id , rvfi_rs3_addr_ex     , rvfi_rs3_addr_wb      ;
    reg [31:0] rvfi_rs3_rdata_id, rvfi_rs3_rdata_ex    , rvfi_rs3_rdata_wb     ;
    reg [ 4:0]                    rvfi_post_rd_addr_ex , rvfi_post_rd_addr_wb  ;
    reg [31:0]                    rvfi_post_rd_wdata_ex, rvfi_post_rd_wdata_wb ;
`endif
                        
    wire insn_id_is_pri_load = rvfi_insn_id[6:0] == OPCODE_CUSTOM_0 
                            && rvfi_insn_id[14:12] inside {3'h0, 3'h4, 3'h1, 3'h5, 3'h2}; // Post-Increment Register-Immediate Load
    wire insn_id_is_prr_load = rvfi_insn_id[6:0] == OPCODE_CUSTOM_1 
                            && rvfi_insn_id[31:25] inside {7'h0, 7'h8, 7'h1, 7'h9, 7'h2} 
                            && rvfi_insn_id[14:12] == 3'b011; // Post-Increment Register-Register Load
    wire insn_id_is_rr_load  = rvfi_insn_id[6:0] == OPCODE_CUSTOM_1 
                            && rvfi_insn_id[31:25] inside {7'h4, 7'hC, 7'h5, 7'hD, 7'h6} 
                            && rvfi_insn_id[14:12] == 3'b011; // Register-Register Load
    wire insn_id_is_p_load   = insn_id_is_pri_load || insn_id_is_prr_load || insn_id_is_rr_load; //Post-Increment Load
                        
    wire insn_id_is_pri_store = rvfi_insn_id[6:0] == OPCODE_CUSTOM_1 
                             && rvfi_insn_id[14:12] inside {3'h0, 3'h1, 3'h2}; // Post-Increment Register-Immediate Store
    wire insn_id_is_prr_store = rvfi_insn_id[6:0] == OPCODE_CUSTOM_1 
                             && rvfi_insn_id[31:25] inside {7'h10, 7'h11, 7'h12} 
                             && rvfi_insn_id[14:12] == 3'b011; // Post-Increment Register-Register Store
    wire insn_id_is_rr_store = rvfi_insn_id[6:0] == OPCODE_CUSTOM_1 
                            && rvfi_insn_id[31:25] inside {7'h14, 7'h15, 7'h16} 
                            && rvfi_insn_id[14:12] == 3'b011; // Register-Register Store
    wire insn_id_is_p_store  = insn_id_is_pri_store || insn_id_is_prr_store || insn_id_is_rr_store; //Post-Increment Store
    
    wire insn_id_is_post = insn_id_is_p_load || insn_id_is_p_store;
    
    logic [31:0] aux_csr_rd_wdata;
    logic [ 4:0] aux_csr_rd_addr;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_rs1_addr_id <= '0;
            rvfi_rs1_addr_ex <= '0;
            rvfi_rs1_addr_wb <= '0;
            rvfi_rs1_rdata_id <= '0;
            rvfi_rs1_rdata_ex <= '0;
            rvfi_rs1_rdata_wb <= '0;
        end
        else begin
            if (id_stage_i.id_ready_o) begin
                if (id_stage_i.rega_used_dec) begin
                    rvfi_rs1_addr_id <= id_stage_i.regfile_addr_ra_id;
                    rvfi_rs1_rdata_id <= id_stage_i.operand_a_fw_id;
                end
                else 
                begin
                    rvfi_rs1_addr_id <= '0;
                    rvfi_rs1_rdata_id <= '0;
                end
            end
            if (ex_stage_i.ex_ready_o) begin
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                begin
                    rvfi_rs1_addr_ex <= rvfi_rs1_addr_id;
                    rvfi_rs1_rdata_ex <= rvfi_rs1_rdata_id;
                end
            end
            if (load_store_unit_i.lsu_ready_wb_o) begin
                rvfi_rs1_addr_wb <= rvfi_rs1_addr_ex;
                rvfi_rs1_rdata_wb <= rvfi_rs1_rdata_ex;
            end
        end
    end
    assign rvfi_rs1_addr = rvfi_rs1_addr_wb;
    assign rvfi_rs1_rdata = rvfi_rs1_rdata_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_rs2_addr_id <= '0;
            rvfi_rs2_addr_ex <= '0;
            rvfi_rs2_addr_wb <= '0;
            rvfi_rs2_rdata_id <= '0;
            rvfi_rs2_rdata_ex <= '0;
            rvfi_rs2_rdata_wb <= '0;
        end
        else begin
            if (id_stage_i.id_ready_o) begin
                if (id_stage_i.regb_used_dec) begin
                    rvfi_rs2_addr_id <= id_stage_i.regfile_addr_rb_id;
                    rvfi_rs2_rdata_id <= id_stage_i.operand_b_fw_id;
                end
                else 
                begin
                    rvfi_rs2_addr_id <= '0;
                    rvfi_rs2_rdata_id <= '0;
                end
            end
            if (ex_stage_i.ex_ready_o) begin
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                begin
                    rvfi_rs2_addr_ex <= rvfi_rs2_addr_id;
                    rvfi_rs2_rdata_ex <= rvfi_rs2_rdata_id;
                end
            end
            if (load_store_unit_i.lsu_ready_wb_o) begin
                rvfi_rs2_addr_wb <= rvfi_rs2_addr_ex;
                rvfi_rs2_rdata_wb <= rvfi_rs2_rdata_ex;
            end
        end
    end
    assign rvfi_rs2_addr = rvfi_rs2_addr_wb;
    assign rvfi_rs2_rdata = rvfi_rs2_rdata_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_rd_addr_ex <= '0;
            rvfi_rd_addr_wb <= '0;
            aux_csr_rd_addr <= '0;
        end
        else begin
            // Without this, the interface won't capture CSR rd after mem instr stall
            if (insn_id_is_csr) begin
                if (ex_stage_i.regfile_alu_we_fw_o)
                    aux_csr_rd_addr <= ex_stage_i.regfile_alu_waddr_fw_o;
                else if (rvfi_insn_id != rvfi_insn_id_q)
                    aux_csr_rd_addr <= '0;
            end
            else
            if (ex_stage_i.ex_ready_o) begin
                // Update rd addr if we is asserted
                if (!insn_id_is_post && ex_stage_i.regfile_alu_we_fw_o)
                    rvfi_rd_addr_ex <= ex_stage_i.regfile_alu_waddr_fw_o;
                else
                    rvfi_rd_addr_ex <= '0;
            end
            
            if (load_store_unit_i.lsu_ready_wb_o) begin
                if (insn_ex_is_csr)
                    rvfi_rd_addr_wb <= aux_csr_rd_addr;
                else
                    // Also check for we in WB stage
                    rvfi_rd_addr_wb <= (id_stage_i.regfile_we_wb_i) ? id_stage_i.regfile_waddr_wb_i : rvfi_rd_addr_ex;
            end
        end
    end
    assign rvfi_rd_addr = rvfi_rd_addr_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_rd_wdata_ex <= '0;
            rvfi_rd_wdata_wb <= '0;
            aux_csr_rd_wdata <= '0;
        end
        else begin
            // Without this, the interface won't capture CSR rd after mem instr stall
            if (insn_id_is_csr) begin
                if (ex_stage_i.regfile_alu_we_fw_o)
                    if (ex_stage_i.regfile_alu_waddr_fw_o == '0)
                        aux_csr_rd_wdata <= '0;
                    else 
                        aux_csr_rd_wdata <= ex_stage_i.regfile_alu_wdata_fw_o;
                else if (rvfi_insn_id != rvfi_insn_id_q)
                    aux_csr_rd_wdata <= '0;
            end
            else
            if (ex_stage_i.ex_ready_o) begin
                // Update rd rdata if we is asserted and addr is not 0
                if (ex_stage_i.regfile_alu_we_fw_o && !insn_id_is_post) begin
                    if (ex_stage_i.regfile_alu_waddr_fw_o == '0)
                        rvfi_rd_wdata_ex <= '0;
                    else 
                        rvfi_rd_wdata_ex <= ex_stage_i.regfile_alu_wdata_fw_o;
                end
                else
                    rvfi_rd_wdata_ex <= '0;
            end
            
            if (load_store_unit_i.lsu_ready_wb_o) begin
                if (insn_ex_is_csr)
                    rvfi_rd_wdata_wb <= aux_csr_rd_wdata;
                else
                // Also check for we in WB stage
                if (id_stage_i.regfile_we_wb_i)
                    if (id_stage_i.regfile_waddr_wb_i == '0)
                        rvfi_rd_wdata_wb <= '0;
                    else 
                        rvfi_rd_wdata_wb <= id_stage_i.regfile_wdata_wb_i;
                else
                    rvfi_rd_wdata_wb <= rvfi_rd_wdata_ex;
            end
        end
    end
    assign rvfi_rd_wdata = rvfi_rd_wdata_wb;
    
`ifdef RISCV_FORMAL_CUSTOM_ISA
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_rs3_addr_id <= '0;
            rvfi_rs3_addr_ex <= '0;
            rvfi_rs3_addr_wb <= '0;
            rvfi_rs3_rdata_id <= '0;
            rvfi_rs3_rdata_ex <= '0;
            rvfi_rs3_rdata_wb <= '0;
        end
        else begin
            if (id_stage_i.id_ready_o) begin
                rvfi_rs3_addr_id <= id_stage_i.regfile_addr_rc_id;
                rvfi_rs3_rdata_id <= id_stage_i.operand_c_fw_id;
            end
            if (ex_stage_i.ex_ready_o) begin
                // If a mem instr is waiting for rvalid, don't update
                // if (!stall_ex)
                begin
                    rvfi_rs3_addr_ex <= rvfi_rs3_addr_id;
                    rvfi_rs3_rdata_ex <= rvfi_rs3_rdata_id;
                end
            end
            if (load_store_unit_i.lsu_ready_wb_o) begin
                rvfi_rs3_addr_wb <= rvfi_rs3_addr_ex;
                rvfi_rs3_rdata_wb <= rvfi_rs3_rdata_ex;
            end
        end
    end
    assign rvfi_rs3_addr = rvfi_rs3_addr_wb;
    assign rvfi_rs3_rdata = rvfi_rs3_rdata_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_post_rd_addr_ex <= '0;
            rvfi_post_rd_addr_wb <= '0;
        end
        else begin
            if (ex_stage_i.ex_ready_o) begin
                // If a mem instr is waiting for rvalid, don't update
                // if (!stall_ex)
                    if (insn_id_is_post && ex_stage_i.regfile_alu_we_fw_o) // Only update if post-incr instr is detected
                        rvfi_post_rd_addr_ex <= ex_stage_i.regfile_alu_waddr_fw_o;
                    else if (!insn_id_is_post)
                        rvfi_post_rd_addr_ex <= '0;
            end
            if (load_store_unit_i.lsu_ready_wb_o) begin
                rvfi_post_rd_addr_wb <= rvfi_post_rd_addr_ex;
            end
        end
    end
    assign rvfi_post_rd_addr = rvfi_post_rd_addr_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_post_rd_wdata_ex <= '0;
            rvfi_post_rd_wdata_wb <= '0;
        end
        else begin
            if (ex_stage_i.ex_ready_o) begin
                // If a mem instr is waiting for rvalid, don't update
                // if (!stall_ex)
                    if (insn_id_is_post && ex_stage_i.regfile_alu_we_fw_o) // Only update if post-incr instr is detected
                        if (ex_stage_i.regfile_alu_waddr_fw_o != '0)
                            rvfi_post_rd_wdata_ex <= ex_stage_i.regfile_alu_wdata_fw_o;
                        else
                            rvfi_post_rd_wdata_ex <= '0;
                    else if (!insn_id_is_post)
                        rvfi_post_rd_wdata_ex <= '0;
            end
            if (load_store_unit_i.lsu_ready_wb_o) begin
                rvfi_post_rd_wdata_wb <= rvfi_post_rd_wdata_ex;
            end
        end
    end
    assign rvfi_post_rd_wdata = rvfi_post_rd_wdata_wb;
`endif
    
    
    
    //====================   Program Counter   ====================//
    reg [31:0] rvfi_pc_rdata_if, rvfi_pc_rdata_id, rvfi_pc_rdata_ex, rvfi_pc_rdata_wb;
    reg [31:0]                   rvfi_pc_wdata_id, rvfi_pc_wdata_ex, rvfi_pc_wdata_wb;
`ifdef RISCV_FORMAL_CUSTOM_ISA
    reg        rvfi_is_hwlp_if   , rvfi_is_hwlp_id   , rvfi_is_hwlp_ex   , rvfi_is_hwlp_wb   ;
    reg [31:0] rvfi_hwlp_start_if, rvfi_hwlp_start_id, rvfi_hwlp_start_ex, rvfi_hwlp_start_wb;
`endif
    
    // wire insn_ex_is_branch = rvfi_insn_ex[6:0] == OPCODE_BRANCH 
    //                       && rvfi_insn_ex[14:12] inside {3'h0, 3'h1, 3'h4, 3'h5, 3'h6, 3'h7};
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_pc_rdata_if <= '0;
            rvfi_pc_rdata_id <= '0;
            rvfi_pc_rdata_ex <= '0;
            rvfi_pc_rdata_wb <= '0;
        end
        else begin
            if (if_stage_i.if_ready)
            // if (if_stage_i.if_ready && if_stage_i.if_valid && if_stage_i.instr_valid)
                rvfi_pc_rdata_if <= if_stage_i.pc_if_o;
            if (id_stage_i.id_ready_o)
                rvfi_pc_rdata_id <= rvfi_pc_rdata_if;
            if (ex_stage_i.ex_ready_o)
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                    rvfi_pc_rdata_ex <= rvfi_pc_rdata_id;
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_pc_rdata_wb <= rvfi_pc_rdata_ex;
        end
    end
    assign rvfi_pc_rdata = rvfi_pc_rdata_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_pc_wdata_id <= '0;
            rvfi_pc_wdata_ex <= '0;
            rvfi_pc_wdata_wb <= '0;
        end
        else begin
            if (id_stage_i.branch_taken_ex)
                rvfi_pc_wdata_id <= {ex_stage_i.jump_target_o[31:1], 1'b0};
            else
            if (id_stage_i.id_ready_o)
                // Jumps are taken in ID stage
                if (id_stage_i.ctrl_transfer_insn_in_id inside {BRANCH_JAL, BRANCH_JALR})
                    rvfi_pc_wdata_id <= {id_stage_i.jump_target_o[31:1], 1'b0};
                else
                    // rvfi_pc_wdata_id <= (rvfi_insn_if[1:0]!=2'b11) ? (rvfi_pc_rdata_if + 2) : (rvfi_pc_rdata_if + 4);
                    rvfi_pc_wdata_id <= if_stage_i.pc_if_o;
            
            if (ex_stage_i.ex_ready_o)
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                // Branches are taken in EX stage
                if (id_stage_i.branch_taken_ex)
                    rvfi_pc_wdata_ex <= {ex_stage_i.jump_target_o[31:1], 1'b0};
                else
                    rvfi_pc_wdata_ex <= rvfi_pc_wdata_id;
            
            if (load_store_unit_i.lsu_ready_wb_o)
                // Change below is necessary to acommodate hwloops
    // `ifdef RISCV_FORMAL_CUSTOM_ISA
    //             if (rvfi_is_hwlp_ex)
    //                 rvfi_pc_wdata_wb <= rvfi_hwlp_start_ex;
    //             else
    // `endif
                    rvfi_pc_wdata_wb <= rvfi_pc_wdata_ex;
        end
    end
    assign rvfi_pc_wdata = rvfi_pc_wdata_wb;
    
`ifdef RISCV_FORMAL_CUSTOM_ISA
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_is_hwlp_if <= '0;
            rvfi_is_hwlp_id <= '0;
            rvfi_is_hwlp_ex <= '0;
            rvfi_is_hwlp_wb <= '0;
            rvfi_hwlp_start_if <= '0;
            rvfi_hwlp_start_id <= '0;
            rvfi_hwlp_start_ex <= '0;
            rvfi_hwlp_start_wb <= '0;
        end
        else begin
            `define PREFETCH if_stage_i.prefetch_buffer_i
            
            if (if_stage_i.hwlp_jump) begin
                rvfi_hwlp_start_if <= if_stage_i.hwlp_target;
            end
            rvfi_hwlp_start_id <= rvfi_hwlp_start_if;
            rvfi_hwlp_start_ex <= rvfi_hwlp_start_id;
            rvfi_hwlp_start_wb <= rvfi_hwlp_start_ex;
            
            if (if_stage_i.hwlp_jump) begin
                rvfi_is_hwlp_if <= 1'b1;
            end
            else if (if_stage_i.if_ready) begin
                rvfi_is_hwlp_if <= 1'b0;
            end
            if (id_stage_i.id_ready_o) begin
                rvfi_is_hwlp_id <= rvfi_is_hwlp_if;
            end
            if (ex_stage_i.ex_ready_o) begin
                // If a mem instr is waiting for rvalid, don't update
                // if (!stall_ex)
                    rvfi_is_hwlp_ex <= rvfi_is_hwlp_id;
            end
            if (load_store_unit_i.lsu_ready_wb_o) begin
                rvfi_is_hwlp_wb <= rvfi_is_hwlp_ex;
            end
        end
    end
    assign rvfi_is_hwlp = rvfi_is_hwlp_wb;
    // assign rvfi_is_hwlp = 1'b0; // TODO!!! Fix hardware loops!!
    assign rvfi_hwlp_start = rvfi_hwlp_start_wb;
`endif

    
    //====================   Memory Access   ====================//
    reg [31:0] rvfi_mem_addr_ex , rvfi_mem_addr_wb ;
    reg [ 3:0]                    rvfi_mem_rmask_wb;
    reg [ 3:0] rvfi_mem_wmask_ex, rvfi_mem_wmask_wb;
    reg [31:0]                    rvfi_mem_rdata_wb;
    reg [31:0] rvfi_mem_wdata_ex, rvfi_mem_wdata_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_mem_addr_ex <= '0;
            rvfi_mem_addr_wb <= '0;
        end
        else begin
            if (ex_stage_i.ex_ready_o)
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                    rvfi_mem_addr_ex <= (misaligned_access) ? rvfi_mem_addr_ex : load_store_unit_i.data_addr_o;
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_mem_addr_wb <= rvfi_mem_addr_ex;
        end
    end
    assign rvfi_mem_addr = rvfi_mem_addr_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_mem_rmask_wb <= '0;
        end
        else begin
            case (load_store_unit_i.data_type_q)
                2'b00:       rvfi_mem_rmask_wb = 4'b1111;
                2'b01:       rvfi_mem_rmask_wb = 4'b0011;
                2'b10,2'b11: rvfi_mem_rmask_wb = 4'b0001;
            endcase                
        end
    end
    assign rvfi_mem_rmask = rvfi_mem_rmask_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_mem_wmask_ex <= '0;
            rvfi_mem_wmask_wb <= '0;
        end
        else begin
            if (ex_stage_i.ex_ready_o)
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                // Drive mask if it's a write operation
                if (load_store_unit_i.next_cnt != 0 && load_store_unit_i.data_we_o) begin
                    if (!misaligned_access)
                        rvfi_mem_wmask_ex <= load_store_unit_i.data_be >> load_store_unit_i.data_addr_int[1:0];
                    else 
                        if (load_store_unit_i.data_type_ex_i == 2'b00) // Store word
                            rvfi_mem_wmask_ex <= (load_store_unit_i.data_be << 4-load_store_unit_i.data_addr_int[1:0]) | rvfi_mem_wmask_ex;
                        else if (load_store_unit_i.data_type_ex_i == 2'b01) // Store half-word
                            rvfi_mem_wmask_ex <= (load_store_unit_i.data_be << 1) | rvfi_mem_wmask_ex;
                end
                else // Not a write operation
                    rvfi_mem_wmask_ex <= '0;
            
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_mem_wmask_wb <= rvfi_mem_wmask_ex;
        end
    end
    assign rvfi_mem_wmask = rvfi_mem_wmask_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_mem_rdata_wb <= '0;
        end
        else begin
            rvfi_mem_rdata_wb <= load_store_unit_i.data_rdata_ex_o;
        end
    end
    assign rvfi_mem_rdata = rvfi_mem_rdata_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_mem_wdata_ex <= '0;
            rvfi_mem_wdata_wb <= '0;
        end
        else begin
            if (ex_stage_i.ex_ready_o)
                // Line below is necessary because otherwise it'll skip mem instructions
                // If a mem instr is waiting for rvalid, don't update
                if (!stall_ex)
                    rvfi_mem_wdata_ex <= load_store_unit_i.data_wdata_ex_i;
            if (load_store_unit_i.lsu_ready_wb_o)
                rvfi_mem_wdata_wb <= rvfi_mem_wdata_ex;
        end
    end
    assign rvfi_mem_wdata = rvfi_mem_wdata_wb;

    //====================   CSR - misa   ====================//
    logic [31:0]                    rvfi_csr_mask_wb ;
    logic [31:0] rvfi_csr_rdata_ex, rvfi_csr_rdata_wb;
    logic [31:0] rvfi_csr_wdata_ex, rvfi_csr_wdata_wb;
    
    logic csr_is_h_ex, csr_is_h_wb;
    
    always @(posedge clk or negedge rst_ni) begin
        if (!rst_ni) begin
            rvfi_csr_rdata_ex <= '0;
            rvfi_csr_rdata_wb <= '0;
            rvfi_csr_wdata_ex <= '0;
            rvfi_csr_wdata_wb <= '0;
            csr_is_h_ex <='0;
            csr_is_h_wb <='0;
        end
        else begin
            // if (insn_id_is_csr && cs_registers_i.csr_we_int) begin
            if (insn_id_is_csr) begin
                if (id_stage_i.regfile_alu_we_fw_i)
                    rvfi_csr_rdata_ex <= cs_registers_i.csr_rdata_o;
                rvfi_csr_wdata_ex <= cs_registers_i.csr_wdata_int;
                csr_is_h_ex <= cs_registers_i.csr_addr_i[7];
            end
            if (load_store_unit_i.lsu_ready_wb_o) begin
                rvfi_csr_rdata_wb <= rvfi_csr_rdata_ex;
                rvfi_csr_wdata_wb <= rvfi_csr_wdata_ex;
                csr_is_h_wb <= csr_is_h_ex;
            end
        end
    end
    
    assign rvfi_csr_mask_wb = {32{rvfi_valid_wb}};
    
    assign rvfi_csr_misa_rmask = '1;
    assign rvfi_csr_misa_rdata = cs_registers_i.MISA_VALUE;
    assign rvfi_csr_misa_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_misa_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mstatus   ====================//
    
    assign rvfi_csr_mstatus_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mstatus_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mstatus_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mstatus_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mtvec   ====================//

    assign rvfi_csr_mtvec_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mtvec_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mtvec_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mtvec_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mepc   ====================//

    assign rvfi_csr_mepc_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mepc_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mepc_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mepc_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mcause   ====================//

    assign rvfi_csr_mcause_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mcause_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mcause_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mcause_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mhartid   ====================//

    assign rvfi_csr_mhartid_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mhartid_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mhartid_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mhartid_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mvendorid   ====================//
    
    assign rvfi_csr_mvendorid_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mvendorid_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mvendorid_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mvendorid_wdata = rvfi_csr_wdata_wb;
    
    //====================   CSR - marchid   ====================//

    assign rvfi_csr_marchid_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_marchid_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_marchid_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_marchid_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mimpid   ====================//

    assign rvfi_csr_mimpid_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mimpid_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mimpid_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mimpid_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mie   ====================//

    assign rvfi_csr_mie_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mie_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mie_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mie_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mip   ====================//

    assign rvfi_csr_mip_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mip_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mip_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mip_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mscratch   ====================//

    assign rvfi_csr_mscratch_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mscratch_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mscratch_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mscratch_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mcountinhibit   ====================//

    assign rvfi_csr_mcountinhibit_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mcountinhibit_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mcountinhibit_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mcountinhibit_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mhpmevent3   ====================//

    assign rvfi_csr_mhpmevent3_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mhpmevent3_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mhpmevent3_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mhpmevent3_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mhpmevent31   ====================//

    assign rvfi_csr_mhpmevent31_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mhpmevent31_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_mhpmevent31_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_mhpmevent31_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - mcycle   ====================//
    
    assign rvfi_csr_mcycle_rmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_mcycle_rdata = csr_is_h_wb ? {rvfi_csr_rdata_wb, 32'b0} : {32'b0, rvfi_csr_rdata_wb};
    assign rvfi_csr_mcycle_wmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_mcycle_wdata = csr_is_h_wb ? {rvfi_csr_wdata_wb, 32'b0} : {32'b0, rvfi_csr_wdata_wb};

    //====================   CSR - minstret   ====================//

    assign rvfi_csr_minstret_rmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_minstret_rdata = csr_is_h_wb ? {rvfi_csr_rdata_wb, 32'b0} : {32'b0, rvfi_csr_rdata_wb};
    assign rvfi_csr_minstret_wmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_minstret_wdata = csr_is_h_wb ? {rvfi_csr_wdata_wb, 32'b0} : {32'b0, rvfi_csr_wdata_wb};

    //====================   CSR - mhpmcounter3   ====================//

    assign rvfi_csr_mhpmcounter3_rmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_mhpmcounter3_rdata = csr_is_h_wb ? {rvfi_csr_rdata_wb, 32'b0} : {32'b0, rvfi_csr_rdata_wb};
    assign rvfi_csr_mhpmcounter3_wmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_mhpmcounter3_wdata = csr_is_h_wb ? {rvfi_csr_wdata_wb, 32'b0} : {32'b0, rvfi_csr_wdata_wb};

    //====================   CSR - mhpmcounter31   ====================//

    assign rvfi_csr_mhpmcounter31_rmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_mhpmcounter31_rdata = csr_is_h_wb ? {rvfi_csr_rdata_wb, 32'b0} : {32'b0, rvfi_csr_rdata_wb};
    assign rvfi_csr_mhpmcounter31_wmask = csr_is_h_wb ? {rvfi_csr_mask_wb, 32'b0} : {32'b0, rvfi_csr_mask_wb};
    assign rvfi_csr_mhpmcounter31_wdata = csr_is_h_wb ? {rvfi_csr_wdata_wb, 32'b0} : {32'b0, rvfi_csr_wdata_wb};

`ifdef RISCV_FORMAL_CUSTOM_ISA
    //====================   CSR - hwlp_start0   ====================//

    // assign rvfi_csr_hwlp_start0_rmask = '1;
    // assign rvfi_csr_hwlp_start0_rdata = hwlp_start0_wb;
    assign rvfi_csr_hwlp_start0_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_start0_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_hwlp_start0_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_start0_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - hwlp_start1   ====================//

    assign rvfi_csr_hwlp_start1_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_start1_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_hwlp_start1_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_start1_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - hwlp_end0   ====================//

    assign rvfi_csr_hwlp_end0_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_end0_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_hwlp_end0_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_end0_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - hwlp_end1   ====================//

    assign rvfi_csr_hwlp_end1_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_end1_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_hwlp_end1_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_end1_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - hwlp_counter0   ====================//

    assign rvfi_csr_hwlp_counter0_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_counter0_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_hwlp_counter0_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_counter0_wdata = rvfi_csr_wdata_wb;

    //====================   CSR - hwlp_counter1   ====================//

    assign rvfi_csr_hwlp_counter1_rmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_counter1_rdata = rvfi_csr_rdata_wb;
    assign rvfi_csr_hwlp_counter1_wmask = rvfi_csr_mask_wb;
    assign rvfi_csr_hwlp_counter1_wdata = rvfi_csr_wdata_wb;

`endif
`endif

endmodule
