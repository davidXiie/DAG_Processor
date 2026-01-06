
`ifndef DATAPATH_TOP         //条件编译头文件保护（Header Guard）。防止在大型工程中由于多次包含此文件而导致的模块重复定义错误 。
  `define DATAPATH_TOP

`include "common_pkg.sv"            //引入外部包文件。这些包里定义了全局常量（如位宽、Bank 数量）以及自定义的数据类型（Struct/Enum）
`include "utils_pkg.sv"
`include "instr_decd_pkg.sv"       
`include "module_library.sv"

`include "alu_trees.sv"                //引入各个子模块的实现文件
`include "pipelined_data.sv"
`include "write_back_logic.sv"
 
`ifdef SRAM_RF                          //编译条件分支
  `include "register_banks_sram.sv"
`else
  `include "register_banks.sv"
`endif


module datapath_top (
  input clk, rst, // 时钟和复位信号 

  // 控制信号输入（由指令译码器产生）
  input alu_mode_t      alu_mode     , // 决定 ALU 执行加法还是乘法 
  input crossbar_sel_t  crossbar_sel , // 交叉开关路由选择信号 
  input                 pipe_en      , // 全局流水线使能信号 
  input [N_BANKS - 1 : 0] crossbar_flop_en, // 交叉开关输出寄存器的使能开关 
  input [N_BANKS - 1 : 0] crossbar_pipe_en, // 交叉开关流水线控制信号 
  input reg_wr_mode_t   reg_wr_mode  , // 寄存器写入模式（如自动分配地址模式） 
  input reg_wr_sel_t    reg_wr_sel   , // 写入源选择（选自 ALU、内存或 Crossbar） [cite: 572]
  input reg_rd_addr_t   reg_rd_addr  , // 寄存器读取地址 [cite: 572]
  input reg_we_t        reg_we       , // 寄存器写使能 [cite: 572]
  input reg_re_t        reg_re       , // 寄存器读使能 [cite: 572]
  input reg_inv_t       reg_inv      , // 寄存器有效位清除信号（用于论文提到的有效位管理） [cite: 572]

  // 数据接口
  output word_t [N_BANKS - 1 : 0] mem_inputs, // 定义写入内存的数据 [cite: 572]
  input word_t [N_BANKS - 1 : 0] mem_outputs  // 从内存读取的数据 [cite: 572]
);
  
  // Signals 内部信号声明
  word_t [N_BANKS - 1: 0] reg_outputs;    // 从寄存器组读取的数据
  word_t [N_BANKS - 1: 0] reg_inputs;     // 写入寄存器组的数据
  word_t [N_BANKS - 1: 0] crossbar_outputs_d;     // 交叉开关的直接输出数据
  word_t [N_BANKS - 1: 0] crossbar_outputs_q;     // 交叉开关的寄存器输出数据
  word_t [N_BANKS - 1: 0] crossbar_outputs_piped;   // 交叉开关的流水线输出数据
  logic [N_TREE - 1 : 0] [N_ALU_PER_TREE - 1 : 0] alu_outputs_vld;    // ALU 输出有效位信号
  word_t [N_TREE - 1 : 0] [N_ALU_PER_TREE - 1 : 0] alu_outputs;      // ALU 输出数据
  word_t [N_TREE - 1 : 0] [N_ALU_PER_TREE - 1 : 0] alu_outputs_piped;   // ALU 流水线输出数据
  word_t [N_BANKS - 1 : 0] mem_outputs_piped;       // 内存流水线输出数据

  // Instances
  register_banks register_banks_ins(       //寄存器组模块实例化
    .clk        (clk         ), 
    .rst        (rst         ), 
    .pipe_en    (pipe_en     ), 
    .inputs     (reg_inputs  ), 
    .reg_we     (reg_we      ), 
    .reg_re     (reg_re      ), 
    .reg_inv    (reg_inv     ), 
    .reg_rd_addr(reg_rd_addr ), 
    .outputs    (reg_outputs )
  ); 

  crossbar #(.WORD_LEN(BIT_L), .IN_PORTS(N_BANKS), .OUT_PORTS(N_BANKS)) crossbar_ins (.input_words(reg_outputs), .sel(crossbar_sel), .output_words(crossbar_outputs_d));
                                                 //输入互联网络实例化
  processing_block processing_block_ins(      //算术逻辑单元树模块实例化
    .clk     (clk               ), 
    .rst     (rst               ), 
    .en      (pipe_en   ), 
    .inputs  (crossbar_outputs_q), 
    .alu_mode(alu_mode  ), 
    .outputs_vld (alu_outputs_vld),
    .outputs (alu_outputs       )
  );

  pipelined_data pipelined_data_ins (              //流水线寄存模块实例化
    .clk                  (clk                   ), 
    .rst                  (rst                   ), 
    .pipe_en              (pipe_en               ), 
    .crossbar_pipe_en (crossbar_pipe_en),
    .in_alu_trees_results (alu_outputs           ), 
    .in_alu_trees_results_vld(alu_outputs_vld    ), 
    .in_crossbar_output   (crossbar_outputs_q    ), 
    .out_crossbar_output  (crossbar_outputs_piped), 
    .mem_inputs   (mem_inputs), 
    .out_alu_trees_results(alu_outputs_piped     )
  );
    
  write_back_logic write_back_logic_ins(              //写回逻辑模块实例化
    .reg_we      (reg_we), 
    .reg_wr_sel  (reg_wr_sel             ), 
    .reg_wr_mode (reg_wr_mode            ), 
    .alu_results (alu_outputs_piped      ), 
    .memory_out  (mem_outputs_piped      ), 
    .crossbar_out(crossbar_outputs_piped ), 
    .reg_inputs  (reg_inputs             ) 
  );

  // Logic if needed           
  assign mem_outputs_piped = mem_outputs;           //把输入端口 mem_outputs 连接到内部信号 mem_outputs_piped
  /* assign mem_inputs        = reg_outputs; */

  always_ff @(posedge clk) begin      //时钟上升沿执行
    if (rst== RESET_STATE) begin       //复位状态下将所有 crossbar_outputs_q 数组元素清零
      foreach ( crossbar_outputs_q[i]) begin
        crossbar_outputs_q[i] <= 0;
      end
    end else begin
      foreach ( crossbar_outputs_q[i]) begin
        if (crossbar_flop_en[i] == 1) begin
          crossbar_outputs_q[i] <= crossbar_outputs_d[i];
        end
      end
    end
  end
endmodule

`endif //DATAPATH_TOP
