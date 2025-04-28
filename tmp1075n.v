`timescale 1ns/1ns

module tmp1075n(
    input clk,
    input rst_n,
    input scl,
    inout sda
);

// 寄存器定义
reg [15:0] config_reg = 16'h61A0; // 默认配置值
reg [15:0] temp_reg = 16'h2720;   // 默认温度值(27.20°C)

// I2C状态机参数
localparam IDLE = 0;
localparam ADDR = 1;
localparam REG_ADDR = 2;
localparam WRITE_DATA = 3;
localparam READ_DATA = 4;
localparam ACK = 5;
localparam WAIT_ACK = 6;  // 新增等待主机应答状态

// 内部信号
reg [2:0] state = IDLE;
reg [7:0] shift_reg;
reg [3:0] bit_cnt;  // 修改为4位以支持更多位计数
reg [7:0] reg_addr;
reg rw_bit;
reg ack_out = 1'b1;
reg sda_out = 1'b1;
reg sda_oe = 1'b0;
reg [1:0] byte_cnt = 0;  // 字节计数器，用于跟踪读写的字节数

// SDA线控制
assign sda = sda_oe ? sda_out : 1'bz;

// 主状态机
always @(posedge scl or negedge rst_n) begin
    if(!rst_n) begin
        state <= IDLE;
        sda_oe <= 1'b0;
        ack_out <= 1'b1;
        byte_cnt <= 0;
    end
    else begin
        case(state)
            IDLE: begin
                sda_oe <= 1'b0;  // 释放SDA总线
                if(!sda) begin // 检测起始条件
                    state <= ADDR;
                    bit_cnt <= 7;
                    byte_cnt <= 0;
                end
            end
            
            ADDR: begin
                sda_oe <= 1'b0;  // 确保在接收地址时不驱动SDA
                if(bit_cnt > 0) begin
                    shift_reg[bit_cnt-1] <= sda;
                    bit_cnt <= bit_cnt - 1;
                end
                else begin
                    rw_bit <= sda;
                    // 检查地址是否匹配(0x48)
                    if(shift_reg[6:0] == 7'b1001000) begin
                        state <= ACK;
                        ack_out <= 1'b0; // 发送ACK（低电平）
                    end
                    else begin
                        state <= IDLE; // 地址不匹配
                    end
                end
            end
            
            ACK: begin
                sda_oe <= 1'b1;  // 从机控制SDA发送应答
                sda_out <= 1'b0; // 明确拉低SDA输出，表示ACK
                if(rw_bit && byte_cnt == 0) begin // 读操作且是第一个字节
                    state <= READ_DATA;
                    bit_cnt <= 7;
                    byte_cnt <= byte_cnt + 1;
                end
                else if(!rw_bit) begin // 写操作
                    if(byte_cnt == 0) begin
                        state <= REG_ADDR;  // 接收寄存器地址
                        bit_cnt <= 7;
                        byte_cnt <= byte_cnt + 1;
                    end
                    else begin
                        state <= WRITE_DATA;  // 接收数据
                        bit_cnt <= 7;
                        byte_cnt <= byte_cnt + 1;
                    end
                end
                else begin
                    state <= IDLE;  // 其他情况回到空闲状态
                end
            end
            
            REG_ADDR: begin
                sda_oe <= 1'b0;  // 释放SDA总线接收数据
                if(bit_cnt > 0) begin
                    shift_reg[bit_cnt-1] <= sda;
                    bit_cnt <= bit_cnt - 1;
                end
                else begin
                    reg_addr <= shift_reg;
                    state <= ACK;
                    ack_out <= 1'b0;  // 发送ACK（低电平）
                end
            end
            
            WRITE_DATA: begin
                sda_oe <= 1'b0;  // 释放SDA总线接收数据
                if(bit_cnt > 0) begin
                    shift_reg[bit_cnt-1] <= sda;
                    bit_cnt <= bit_cnt - 1;
                end
                else begin
                    // 写入配置寄存器
                    if(reg_addr == 8'h01) begin
                        if(byte_cnt == 2) // 第一个数据字节是高8位
                            config_reg[15:8] <= shift_reg;
                        else if(byte_cnt == 3) // 第二个数据字节是低8位
                            config_reg[7:0] <= shift_reg;
                    end
                    state <= ACK;
                    ack_out <= 1'b0;  // 发送ACK（低电平）
                end
            end
            
            READ_DATA: begin
                sda_oe <= 1'b1;  // 从机控制SDA发送数据
                if(reg_addr == 8'h00) begin // 温度寄存器
                    if(byte_cnt == 1) // 第一个字节（高字节）
                        sda_out <= temp_reg[15-bit_cnt];
                    else if(byte_cnt == 2) // 第二个字节（低字节）
                        sda_out <= temp_reg[7-bit_cnt];
                end
                
                if(bit_cnt > 0) begin
                    bit_cnt <= bit_cnt - 1;
                end
                else begin
                    state <= WAIT_ACK;  // 等待主机应答
                    sda_oe <= 1'b0;  // 释放SDA总线
                    byte_cnt <= byte_cnt + 1;
                end
            end
            
            WAIT_ACK: begin
                sda_oe <= 1'b0;  // 释放SDA总线等待主机应答
                // 检查主机应答
                if(sda == 1'b1) begin // 主机不应答（NACK），结束传输
                    state <= IDLE;
                end
                else begin // 主机应答（ACK），继续传输
                    state <= READ_DATA;
                    bit_cnt <= 7;
                end
            end
        endcase
    end
end

// 检测起始和停止条件
always @(posedge sda) begin
    if(scl) begin // 停止条件：SCL高时SDA从低变高
        state <= IDLE;
        sda_oe <= 1'b0;
        byte_cnt <= 0;
    end
end

always @(negedge sda) begin
    if(scl && state == IDLE) begin // 起始条件：SCL高时SDA从高变低
        state <= ADDR;
        bit_cnt <= 7;
        byte_cnt <= 0;
    end
end

endmodule