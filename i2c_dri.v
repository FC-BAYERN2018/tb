//****************************************Copyright (c)***********************************/
// 说明：IIC控制器，具有以下特点
//----------------------------------------------------------------------------------------
// File name:           i2c_dri
// Last modified Date:  2019/05/04 9:19:08
// Last Version:        V1.0
// Descriptions:        IIC控制器
//                      
//----------------------------------------------------------------------------------------
// Created by:          LiuYu
// Created date:        2019/05/04 9:19:08
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module i2c_dri
    #(
      parameter   SLAVE_ADDR = 7'b1010000   ,  //EEPROM从设备地址
      parameter   CLK_FREQ   = 26'd50_000_000, //系统时钟频率
      parameter   I2C_FREQ   = 18'd250_000     //IIC_SCL时钟频率
    )
   (                                                    
    input                clk        ,    
    input                rst_n      ,   
                                         
    //i2c interface                                      
    input                i2c_exec   ,  //I2C执行控制信号
    input                bit_ctrl   ,  //控制地址位数（16位/8位）
    input                i2c_rh_wl  ,  //I2C读写控制信号
    input        [15:0]  i2c_addr   ,  //I2C设备内部地址
    input        [ 7:0]  i2c_data_w ,  //I2C要写入的数据
    output  reg  [ 7:0]  i2c_data_r ,  //I2C读取的数据
    output  reg          i2c_done   ,  //I2C一次操作完成标志
    output  reg          i2c_ack    ,  //I2C应答信号 0:应答 1:未应答
    output  reg          scl        ,  //I2C时钟SCL输出
    inout                sda        ,  //I2C数据SDA双向信号
                                       
    //user interface                                   
    output  reg          dri_clk       //I2C控制时钟信号
     );

//localparam define
localparam  st_idle     = 8'd1; //空闲状态
localparam  st_sladdr   = 8'd2; //发送从设备地址状态
localparam  st_addr16   = 8'd3; //发送16位内部地址状态
localparam  st_addr8    = 8'd4; //发送8位内部地址状态
localparam  st_data_wr  = 8'd5; //写数据状态（8位）
localparam  st_addr_rd  = 8'd6; //发送读命令状态
localparam  st_data_rd  = 8'd7; //读取数据状态（8位）
localparam  st_stop     = 8'd8; //停止I2C传输状态

//reg define
reg            sda_dir   ; //I2C数据线（SDA）方向控制
reg            sda_out   ; //SDA输出缓冲
reg            st_done   ; //状态完成标志
reg            wr_flag   ; //写标志
reg    [ 6:0]  cnt       ; //计数器
reg    [ 7:0]  cur_state ; //当前状态
reg    [ 7:0]  next_state; //下一个状态
reg    [15:0]  addr_t    ; //地址缓冲
reg    [ 7:0]  data_r    ; //读取数据缓冲
reg    [ 7:0]  data_wr_t ; //I2C写数据缓冲，在时序调整时使用
reg    [ 9:0]  clk_cnt   ; //时钟分频计数器

//wire define
wire          sda_in     ; //SDA输入缓冲
wire   [8:0]  clk_divide ; //系统时钟分频计算得到I2C时钟周期的系数

//*****************************************************
//**                    main code
//*****************************************************

//SDA三态输出控制
assign  sda     = sda_dir ?  sda_out : 1'bz;     //SDA输出控制
assign  sda_in  = sda ;                          //SDA输入缓冲
assign  clk_divide = (CLK_FREQ/I2C_FREQ) >> 2'd2;//系统时钟分频得到I2C时钟周期的系数

//产生I2C时钟SCL，通过分频系统时钟得到I2C时钟周期
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        dri_clk <=  1'b0;
        clk_cnt <= 10'd0;
    end
    else if(clk_cnt == clk_divide[8:1] - 1'd1) begin
        clk_cnt <= 10'd0;
        dri_clk <= ~dri_clk;
    end
    else
        clk_cnt <= clk_cnt + 1'b1;
end

//I2C传输状态机，同步时钟为dri_clk
always @(posedge dri_clk or negedge rst_n) begin
    if(!rst_n)
        cur_state <= st_idle;
    else
        cur_state <= next_state;
end

//状态转移逻辑
always @(*) begin
    next_state = st_idle;
    case(cur_state)
        st_idle: begin                          //空闲状态
           if(i2c_exec) begin
               next_state = st_sladdr;
           end
           else
               next_state = st_idle;
        end
        st_sladdr: begin
            if(st_done) begin
                if(bit_ctrl)                    //控制信号决定是否发送16位或8位内部地址
                   next_state = st_addr16;
                else
                   next_state = st_addr8 ;
            end
            else
                next_state = st_sladdr;
        end
        st_addr16: begin                        //发送16位内部地址
            if(st_done) begin
                next_state = st_addr8;
            end
            else begin
                next_state = st_addr16;
            end
        end
        st_addr8: begin                         //发送8位内部地址
            if(st_done) begin
                if(wr_flag==1'b0)               //判断是写操作还是读操作
                    next_state = st_data_wr;
                else
                    next_state = st_addr_rd;
            end
            else begin
                next_state = st_addr8;
            end
        end
        st_data_wr: begin                       //写数据（8位）
            if(st_done)
                next_state = st_stop;
            else
                next_state = st_data_wr;
        end
        st_addr_rd: begin                       //发送读命令
            if(st_done) begin
                next_state = st_data_rd;
            end
            else begin
                next_state = st_addr_rd;
            end
        end
        st_data_rd: begin                       //读取数据（8位）
            if(st_done)
                next_state = st_stop;
            else
                next_state = st_data_rd;
        end
        st_stop: begin                          //停止I2C传输
            if(st_done)
                next_state = st_idle;
            else
                next_state = st_stop ;
        end
        default: next_state= st_idle;
    endcase
end

//每个状态的具体操作和时序控制
always @(posedge dri_clk or negedge rst_n) begin
    //复位时初始化所有信号
    if(!rst_n) begin
        scl       <= 1'b1;
        sda_out   <= 1'b1;
        sda_dir   <= 1'b1;                          
        i2c_done  <= 1'b0;                          
        i2c_ack   <= 1'b0;                          
        cnt       <= 1'b0;                          
        st_done   <= 1'b0;                          
        data_r    <= 1'b0;                          
        i2c_data_r<= 1'b0;                          
        wr_flag   <= 1'b0;                          
        addr_t    <= 1'b0;                          
        data_wr_t <= 1'b0;                          
    end                                              
    else begin                                       
        st_done <= 1'b0 ;                             //默认状态完成标志为低
        cnt     <= cnt +1'b1 ;                        //计数器递增
        case(cur_state)                               //根据当前状态执行不同操作
             st_idle: begin                          //空闲状态，等待启动信号
                scl     <= 1'b1;                      //保持时钟高电平
                sda_out <= 1'b1;                      //保持数据线高电平
                sda_dir <= 1'b1;                      //SDA为输出模式
                i2c_done<= 1'b0;                      //清除完成标志
                cnt     <= 7'b0;                      //复位计数器
                if(i2c_exec) begin                    //接收到启动信号后初始化参数
                    wr_flag   <= i2c_rh_wl ;          //设置读写标志
                    addr_t    <= i2c_addr  ;          //加载内部地址
                    data_wr_t <= i2c_data_w;          //加载写数据
                    i2c_ack <= 1'b0;                  //清除应答标志
                end                                   //等待启动信号
            end                                       //空闲状态结束
            st_sladdr: begin                         //发送从设备地址状态
                case(cnt)                             //根据计数器执行不同步骤
                    7'd1 : sda_out <= 1'b0;          //拉低SDA产生I2C起始条件
                    7'd3 : scl <= 1'b0;              
                    7'd4 : sda_out <= SLAVE_ADDR[6]; //发送从设备地址第7位
                    7'd5 : scl <= 1'b1;              
                    7'd7 : scl <= 1'b0;              
                    7'd8 : sda_out <= SLAVE_ADDR[5]; //发送从设备地址第6位
                    7'd9 : scl <= 1'b1;              
                    7'd11: scl <= 1'b0;              
                    7'd12: sda_out <= SLAVE_ADDR[4]; //发送从设备地址第5位
                    7'd13: scl <= 1'b1;              
                    7'd15: scl <= 1'b0;              
                    7'd16: sda_out <= SLAVE_ADDR[3]; //发送从设备地址第4位
                    7'd17: scl <= 1'b1;              
                    7'd19: scl <= 1'b0;              
                    7'd20: sda_out <= SLAVE_ADDR[2]; //发送从设备地址第3位
                    7'd21: scl <= 1'b1;              
                    7'd23: scl <= 1'b0;              
                    7'd24: sda_out <= SLAVE_ADDR[1]; //发送从设备地址第2位
                    7'd25: scl <= 1'b1;              
                    7'd27: scl <= 1'b0;              
                    7'd28: sda_out <= SLAVE_ADDR[0]; //发送从设备地址第1位
                    7'd29: scl <= 1'b1;              
                    7'd31: scl <= 1'b0;              
                    7'd32: sda_out <= 1'b0;          //设置读写位（0：写）
                    7'd33: scl <= 1'b1;              
                    7'd35: scl <= 1'b0;              
                    7'd36: begin                     
                        sda_dir <= 1'b0;              //切换SDA为输入模式
                        sda_out <= 1'b1;             
                    end                              
                    7'd37: scl     <= 1'b1;            //拉高时钟
                    7'd38: begin                       //检测从设备应答信号
                        st_done <= 1'b1;              //设置状态完成标志
                        if(sda_in == 1'b1)            //如果SDA为高电平，表示未收到应答
                            i2c_ack <= 1'b1;          //设置未应答标志
                    end                              
                    7'd39: begin                       
                        scl <= 1'b0;                  //拉低时钟
                        cnt <= 1'b0;                  //复位计数器
                    end                               
                    default :  ;                      //其他情况保持不变
                endcase                               
            end                                       
            st_addr16: begin                         
                case(cnt)                             
                    7'd0 : begin                      
                        sda_dir <= 1'b1 ;            
                        sda_out <= addr_t[15];       //发送16位内部地址第16位
                    end                               
                    7'd1 : scl <= 1'b1;              
                    7'd3 : scl <= 1'b0;              
                    7'd4 : sda_out <= addr_t[14];    //发送16位内部地址第15位
                    7'd5 : scl <= 1'b1;              
                    7'd7 : scl <= 1'b0;              
                    7'd8 : sda_out <= addr_t[13];    //发送16位内部地址第14位
                    7'd9 : scl <= 1'b1;              
                    7'd11: scl <= 1'b0;              
                    7'd12: sda_out <= addr_t[12];    //发送16位内部地址第13位
                    7'd13: scl <= 1'b1;              
                    7'd15: scl <= 1'b0;              
                    7'd16: sda_out <= addr_t[11];    //发送16位内部地址第12位
                    7'd17: scl <= 1'b1;              
                    7'd19: scl <= 1'b0;              
                    7'd20: sda_out <= addr_t[10];    //发送16位内部地址第11位
                    7'd21: scl <= 1'b1;              
                    7'd23: scl <= 1'b0;              
                    7'd24: sda_out <= addr_t[9];     //发送16位内部地址第10位
                    7'd25: scl <= 1'b1;              
                    7'd27: scl <= 1'b0;              
                    7'd28: sda_out <= addr_t[8];     //发送16位内部地址第9位
                    7'd29: scl <= 1'b1;              
                    7'd31: scl <= 1'b0;              
                    7'd32: begin                     
                        sda_dir <= 1'b0;             
                        sda_out <= 1'b1;            
                    end                              
                    7'd33: scl  <= 1'b1;             
                    7'd34: begin                     
                        st_done <= 1'b1;     
                        if(sda_in == 1'b1)           //检测从设备应答信号
                            i2c_ack <= 1'b1;         
                    end                             
                    7'd35: begin                     
                        scl <= 1'b0;                 
                        cnt <= 1'b0;                 
                    end                              
                    default :  ;                     
                endcase                               
            end                                       
            st_addr8: begin                           
                case(cnt)                             
                    7'd0: begin                       
                       sda_dir <= 1'b1 ;              
                       sda_out <= addr_t[7];         //发送8位内部地址第8位
                    end                               
                    7'd1 : scl <= 1'b1;              
                    7'd3 : scl <= 1'b0;              
                    7'd4 : sda_out <= addr_t[6];     //发送8位内部地址第7位
                    7'd5 : scl <= 1'b1;              
                    7'd7 : scl <= 1'b0;              
                    7'd8 : sda_out <= addr_t[5];     //发送8位内部地址第6位
                    7'd9 : scl <= 1'b1;              
                    7'd11: scl <= 1'b0;              
                    7'd12: sda_out <= addr_t[4];     //发送8位内部地址第5位
                    7'd13: scl <= 1'b1;              
                    7'd15: scl <= 1'b0;              
                    7'd16: sda_out <= addr_t[3];     //发送8位内部地址第4位
                    7'd17: scl <= 1'b1;              
                    7'd19: scl <= 1'b0;              
                    7'd20: sda_out <= addr_t[2];     //发送8位内部地址第3位
                    7'd21: scl <= 1'b1;              
                    7'd23: scl <= 1'b0;              
                    7'd24: sda_out <= addr_t[1];     //发送8位内部地址第2位
                    7'd25: scl <= 1'b1;              
                    7'd27: scl <= 1'b0;              
                    7'd28: sda_out <= addr_t[0];     //发送8位内部地址第1位
                    7'd29: scl <= 1'b1;              
                    7'd31: scl <= 1'b0;              
                    7'd32: begin                     
                        sda_dir <= 1'b0;             
                        sda_out <= 1'b1;                    
                    end                              
                    7'd33: scl     <= 1'b1;          
                    7'd34: begin                     
                        st_done <= 1'b1;     
                        if(sda_in == 1'b1)           //检测从设备应答信号
                            i2c_ack <= 1'b1;         
                    end                               
                    7'd35: begin                     
                        scl <= 1'b0;                 
                        cnt <= 1'b0;                 
                    end                              
                    default :  ;                     
                endcase                               
            end                                       
            st_data_wr: begin                         //写数据（8位）状态
                case(cnt)                             
                    7'd0: begin                       
                        sda_out <= data_wr_t[7];     //发送写数据第8位
                        sda_dir <= 1'b1;             
                    end                               
                    7'd1 : scl <= 1'b1;              
                    7'd3 : scl <= 1'b0;              
                    7'd4 : sda_out <= data_wr_t[6];  //发送写数据第7位
                    7'd5 : scl <= 1'b1;              
                    7'd7 : scl <= 1'b0;              
                    7'd8 : sda_out <= data_wr_t[5];  //发送写数据第6位
                    7'd9 : scl <= 1'b1;              
                    7'd11: scl <= 1'b0;              
                    7'd12: sda_out <= data_wr_t[4];  //发送写数据第5位
                    7'd13: scl <= 1'b1;              
                    7'd15: scl <= 1'b0;              
                    7'd16: sda_out <= data_wr_t[3];  //发送写数据第4位
                    7'd17: scl <= 1'b1;              
                    7'd19: scl <= 1'b0;              
                    7'd20: sda_out <= data_wr_t[2];  //发送写数据第3位
                    7'd21: scl <= 1'b1;              
                    7'd23: scl <= 1'b0;              
                    7'd24: sda_out <= data_wr_t[1];  //发送写数据第2位
                    7'd25: scl <= 1'b1;              
                    7'd27: scl <= 1'b0;              
                    7'd28: sda_out <= data_wr_t[0];  //发送写数据第1位
                    7'd29: scl <= 1'b1;              
                    7'd31: scl <= 1'b0;              
                    7'd32: begin                     
                        sda_dir <= 1'b0;           
                        sda_out <= 1'b1;                               
                    end                               
                    7'd33: scl <= 1'b1;              
                    7'd34: begin                     
                        st_done <= 1'b1;     
                        if(sda_in == 1'b1)           //检测从设备应答信号
                            i2c_ack <= 1'b1;         
                    end                             
                    7'd35: begin                     
                        scl  <= 1'b0;                
                        cnt  <= 1'b0;                
                    end                              
                    default  :  ;                    
                endcase                               
            end                                       
            st_addr_rd: begin                         //发送读命令状态
                case(cnt)                             
                    7'd0 : begin                      
                        sda_dir <= 1'b1;             
                        sda_out <= 1'b1;             
                    end                               
                    7'd1 : scl <= 1'b1;              
                    7'd2 : sda_out <= 1'b0;          //产生I2C起始条件
                    7'd3 : scl <= 1'b0;              
                    7'd4 : sda_out <= SLAVE_ADDR[6]; //发送从设备地址第7位
                    7'd5 : scl <= 1'b1;              
                    7'd7 : scl <= 1'b0;              
                    7'd8 : sda_out <= SLAVE_ADDR[5]; 
                    7'd9 : scl <= 1'b1;              
                    7'd11: scl <= 1'b0;              
                    7'd12: sda_out <= SLAVE_ADDR[4]; 
                    7'd13: scl <= 1'b1;              
                    7'd15: scl <= 1'b0;              
                    7'd16: sda_out <= SLAVE_ADDR[3]; 
                    7'd17: scl <= 1'b1;              
                    7'd19: scl <= 1'b0;              
                    7'd20: sda_out <= SLAVE_ADDR[2]; 
                    7'd21: scl <= 1'b1;              
                    7'd23: scl <= 1'b0;              
                    7'd24: sda_out <= SLAVE_ADDR[1]; 
                    7'd25: scl <= 1'b1;              
                    7'd27: scl <= 1'b0;              
                    7'd28: sda_out <= SLAVE_ADDR[0]; 
                    7'd29: scl <= 1'b1;              
                    7'd31: scl <= 1'b0;              
                    7'd32: sda_out <= 1'b1;          //设置读写位（1：读）
                    7'd33: scl <= 1'b1;              
                    7'd35: scl <= 1'b0;              
                    7'd36: begin                     
                        sda_dir <= 1'b0;            
                        sda_out <= 1'b1;                    
                    end
                    7'd37: scl     <= 1'b1;
                    7'd38: begin                     
                        st_done <= 1'b1;     
                        if(sda_in == 1'b1)           
                            i2c_ack <= 1'b1;         
                    end   
                    7'd39: begin
                        scl <= 1'b0;
                        cnt <= 1'b0;
                    end
                    default : ;
                endcase
            end
            st_data_rd: begin                         //读取数据（8位）状态
                case(cnt)
                    7'd0: sda_dir <= 1'b0;           //设置SDA为输入模式
                    7'd1: begin
                        data_r[7] <= sda_in;         //读取数据第8位
                        scl       <= 1'b1;
                    end
                    7'd3: scl  <= 1'b0;
                    7'd5: begin
                        data_r[6] <= sda_in ;        //读取数据第7位
                        scl       <= 1'b1   ;
                    end
                    7'd7: scl  <= 1'b0;
                    7'd9: begin
                        data_r[5] <= sda_in;         //读取数据第6位
                        scl       <= 1'b1  ;
                    end
                    7'd11: scl  <= 1'b0;
                    7'd13: begin
                        data_r[4] <= sda_in;         //读取数据第5位
                        scl       <= 1'b1  ;
                    end
                    7'd15: scl  <= 1'b0;
                    7'd17: begin
                        data_r[3] <= sda_in;         //读取数据第4位
                        scl       <= 1'b1  ;
                    end
                    7'd19: scl  <= 1'b0;
                    7'd21: begin
                        data_r[2] <= sda_in;         //读取数据第3位
                        scl       <= 1'b1  ;
                    end
                    7'd23: scl  <= 1'b0;
                    7'd25: begin
                        data_r[1] <= sda_in;         //读取数据第2位
                        scl       <= 1'b1  ;
                    end
                    7'd27: scl  <= 1'b0;
                    7'd29: begin
                        data_r[0] <= sda_in;         //读取数据第1位
                        scl       <= 1'b1  ;
                    end
                    7'd31: scl  <= 1'b0;
                    7'd32: begin
                        sda_dir <= 1'b1;             
                        sda_out <= 1'b1;
                    end
                    7'd33: scl     <= 1'b1;
                    7'd34: st_done <= 1'b1;          //设置状态完成标志
                    7'd35: begin
                        scl <= 1'b0;
                        cnt <= 1'b0;
                        i2c_data_r <= data_r;        //将读取的数据输出
                    end
                    default  :  ;
                endcase
            end
            st_stop: begin                            //停止I2C传输状态
                case(cnt)
                    7'd0: begin
                        sda_dir <= 1'b1;             //设置SDA为输出模式
                        sda_out <= 1'b0;             //拉低SDA
                    end
                    7'd1 : scl     <= 1'b1;          //拉高时钟
                    7'd3 : sda_out <= 1'b1;          //释放SDA（产生停止条件）
                    7'd15: st_done <= 1'b1;          //设置状态完成标志
                    7'd16: begin
                        cnt      <= 1'b0;            //复位计数器
                        i2c_done <= 1'b1;            //设置传输完成标志
                    end
                    default  : ;
                endcase
            end
        endcase
    end
end

endmodule