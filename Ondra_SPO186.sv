//============================================================================
// 
//  Ondra_SPO186 replica for MiST
//  2022 Petr Mrzena
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

`default_nettype none

module Ondra_SPO186
(
   input         CLOCK_27[0],   // Input clock 27 MHz

   output  [5:0] VGA_R,
   output  [5:0] VGA_G,
   output  [5:0] VGA_B,
   output        VGA_HS,
   output        VGA_VS,

   output        LED,

   output        AUDIO_L,
   output        AUDIO_R,

   input         SPI_SCK,
   output        SPI_DO,
   input         SPI_DI,
   input         SPI_SS2,
   input         SPI_SS3,
   input         CONF_DATA0,

   output [12:0] SDRAM_A,
   inout  [15:0] SDRAM_DQ,
   output        SDRAM_DQML,
   output        SDRAM_DQMH,
   output        SDRAM_nWE,
   output        SDRAM_nCAS,
   output        SDRAM_nRAS,
   output        SDRAM_nCS,
   output  [1:0] SDRAM_BA,
   output        SDRAM_CLK,
   output        SDRAM_CKE,

   input         UART_RX,
   output        UART_TX
);


`include "build_id.v"


localparam CONF_STR =
{
    "Ondra_SPO186;;",	
    "O56,ROM,ViLi,Tesla v5,Test ROM;",				
    //"O7,ADC line pass through,On,Off;",
	 //"O8,Ondra SD,On,Off;",
	 //"O9,Ondra Melodik,On,Off;",
    "T0,Reset Ondra;",	
    "V,v",`BUILD_DATE 
};

 

////////////////////   CLOCKS   ///////////////////

wire pll_locked;
wire pll_locked2;
(* keep *) wire clk_sys; // 8MHz clock
(* keep *) wire clk_16;
wire clk_SDRAM; // 16MHz
wire clk_50M;

pll_SDRAM pll_SDRAM
(
    .inclk0(CLOCK_27[0]),
	 .locked(pll_locked),
    .c0(clk_SDRAM)
);


pll pll
(
    .inclk0(CLOCK_27[0]),
	 .locked(pll_locked2),
	 .c0(clk_16),
    .c1(clk_sys),          // 8 MHz
	 .c2(clk_snen)          // 4 MHz 
);


//////////////////   MIST ARM I/O   ///////////////////
wire        key_pressed;  // 1-make (pressed), 0-break (released)
wire        key_extended; // extended code
wire  [7:0] key_code;     // key scan code
wire        key_strobe;   // key data valid

wire  [7:0] joystick_0;
wire  [7:0] joystick_1;
wire  [1:0] buttons;
wire  [1:0] switches;
wire        scandoubler_disable;
wire        ypbpr;
wire        no_csync;

// status word wires
wire [31:0] status;
wire  [1:0] st_rom                 = status[6:5];
//wire        st_ondra_sd            = status[8];
//wire        st_ondra_melodik       = status[9];


user_io #(.STRLEN($size(CONF_STR)>>3)) user_io
(
    .clk_sys(clk_sys),
    .clk_sd(clk_16),
    .SPI_SS_IO(CONF_DATA0),
    .SPI_CLK(SPI_SCK),
    .SPI_MOSI(SPI_DI),
    .SPI_MISO(SPI_DO),

    .conf_str(CONF_STR),

    .status(status),
    .scandoubler_disable(scandoubler_disable),
    .ypbpr(ypbpr),
    .no_csync(no_csync),
    .buttons(buttons),
    .switches(switches),
    .joystick_0(joystick_0),
    .joystick_1(joystick_1),
    .key_pressed(key_pressed),
    .key_extended(key_extended),
    .key_code(key_code),
    .key_strobe(key_strobe),
    
    .sd_lba(sd_lba),
    .sd_rd(sd_rd),
    .sd_wr(sd_wr),
    .sd_ack(sd_ack),
    .sd_conf(sd_conf),
	 .sd_ack_conf(sd_ack_conf),
	 .sd_dout(sd_dout),
    .sd_dout_strobe(sd_dout_strobe),
    .sd_din(sd_din),
    .sd_buff_addr(sd_buff_addr),
//    .sd_conf(0),
    .sd_sdhc(1)
//    .img_mounted(img_mounted),
//    .img_size(img_size)
);
 
//////////////////   Keyboard controls   ////////////////// 
reg old_stb;    
reg kbd_enter = 0;

always @(posedge clk_sys) 
begin
	old_stb <= key_strobe;
   if ((old_stb != key_strobe) & (~key_extended))
	begin		
      case(key_code)
         8'h5a : kbd_enter <= key_pressed;        // ENTER         
      endcase	
   end
end	
 
//////////////////   fake SD-CARD   ////////////////// 
 
 // conections between user_io (implementing the SPIU communication 
// to the io controller) and the legacy 
wire [31:0] sd_lba;
wire sd_rd;
wire sd_wr;
wire sd_ack, sd_ack_conf;
wire sd_conf;
wire sd_sdhc; 
wire [7:0] sd_dout;
wire sd_dout_strobe;
wire [7:0] sd_din;
//wire sd_din_strobe;
wire [8:0] sd_buff_addr;

sd_card sd_card (
	.clk_sys         ( CLOCK_27[0]         ),   // at least 2xsd_sck

	// connection to io controller
	.sd_lba          ( sd_lba         ),
	.sd_rd           ( sd_rd          ),
	.sd_wr           ( sd_wr          ),
	.sd_ack          ( sd_ack         ),
	.sd_conf         ( sd_conf        ),
	.sd_ack_conf     ( sd_ack_conf    ),
	.sd_sdhc         ( sd_sdhc        ),
	.sd_buff_dout    ( sd_dout        ),
	.sd_buff_wr      ( sd_dout_strobe ),
	.sd_buff_din     ( sd_din         ),
	.sd_buff_addr    ( sd_buff_addr   ),

	.allow_sdhc 	( 1'b1            ),   // QLSD supports SDHC

	// connection to local CPU
	.sd_cs   		( SD_CS           ),
	.sd_sck  		( SD_SCK          ),
	.sd_sdi  		( SD_MOSI         ),
	.sd_sdo  		( SD_MISO         )
);
 
 
//////////////////   Ondra SPO core   //////////////////
wire  [7:0] joy = joystick_0 | joystick_1;

wire HSync;
wire VSync;
wire HBlank;
wire VBlank;
wire pixel;
wire beeper;
wire TXD;
wire LED_GREEN;
wire LED_YELLOW;
wire LED_RED;

(* keep *)  wire reset = status[0] | buttons[1] | ~pll_locked | ~pll_locked2;

	 
Ondra_SPO186_core Ondra_SPO186_core
(
//	.clk_50M(clk_50M),
	.clk_50M(CLOCK_27[0]),
	.clk_sys(clk_sys),
	.reset(reset),	
	.ps2_key({key_strobe, key_pressed, key_extended, key_code}),
	.HSync(HSync),
	.VSync(VSync),	
	.HBlank(HBlank),
	.VBlank(VBlank),
	.pixel(pixel),
	.beeper(beeper),
	.LED_GREEN(LED_GREEN),
	.LED_YELLOW(LED_YELLOW),
	.RELAY(LED_RED), // red led will indicate RELAY activity
	.joy(joy),
	.RESERVA_IN(OndraSD_txd), //rxd
	.RESERVA_OUT(OndraSD_rxd), // txd
	.MGF_IN(0),
	.ROMVersion(st_rom),
	.Parallel_Data_OUT(Parallel_Data_OUT),	
   .NON_STB(NON_STB),
	
   .sdram_in(sdram_in),
   .sdram_out(sdram_out),
   .sdram_a(sdram_a),
   .sdram_we(sdram_we),
	.sdram_rd(sdram_rd)	
);

 

//////////////////   MEMORY   //////////////////
assign SDRAM_CLK = clk_SDRAM;

wire  [7:0] sdram_in;	
wire  [7:0] sdram_out;
wire [22:0] sdram_a;
wire        sdram_we;
wire        sdram_rd;
(* keep *) wire sdram_ready;
	
sdram ram
(
	 .*,	 
	 .init(~pll_locked),
	 .clk(clk_SDRAM),
	 .wtbt(2'b11),
	 .dout(sdram_out),
	 .din ({sdram_in, sdram_in}),
	 .addr({sdram_a, 1'b0}),
	 .we(sdram_we),
	 .rd(sdram_rd),
	 .ready(sdram_ready)
);
	

//////////////////   Ondra MELODIK - sn76489_audio   //////////////////
wire [7:0] Parallel_Data_OUT;	
wire NON_STB;
wire clk_snen; // 3.58Mhz
reg  ondra_melodik_clk_enable;
wire [13:0] mix_audio_o;

always @(posedge reset or negedge NON_STB)
begin
  if (reset)
    ondra_melodik_clk_enable <= 0;
  else
    ondra_melodik_clk_enable <= 1;
end

sn76489_audio #(.MIN_PERIOD_CNT_G(17)) sn76489_audio
(  .clk_i(CLOCK_27[0]),                                    //System clock
//.clk_i(clk_16),                                    //System clock
   .en_clk_psg_i(clk_snen & ondra_melodik_clk_enable), //PSG clock enable
	//.en_clk_psg_i(clk_snen), //PSG clock enable
   .ce_n_i(0),                                         //chip enable, active low
   .wr_n_i(NON_STB),                                   // write enable, active low
   .data_i(Parallel_Data_OUT),
   .mix_audio_o(mix_audio_o)
);

 

//////////////////   AUDIO   //////////////////


sigma_delta_dac #(13) dac
(
    .CLK(clk_16),
    .RESET(reset),
    .DACin((beeper ? 14'h0fff : 14'h0000) | mix_audio_o),
	 //.DACin(mix_audio_o),
    .DACout(AUDIO_L)
);

assign AUDIO_R = AUDIO_L;

//////////////////   VIDEO   //////////////////

wire        hs,vs;

mist_video #(.COLOR_DEPTH(4), .OSD_COLOR(3'd5), .SD_HCNT_WIDTH(10)) mist_video (
    .clk_sys     ( clk_16    ),

    // OSD SPI interface
    .SPI_SCK     ( SPI_SCK    ),
    .SPI_SS3     ( SPI_SS3    ),
    .SPI_DI      ( SPI_DI     ),

    // scanlines (00-none 01-25% 10-50% 11-75%)
    .scanlines   ( 2'b00      ),

    // non-scandoubled pixel clock divider 0 - clk_sys/4, 1 - clk_sys/2
    .ce_divider  ( 1'b1       ),

    // 0 = HVSync 31KHz, 1 = CSync 15KHz
    .scandoubler_disable ( scandoubler_disable ),
    // disable csync without scandoubler
    .no_csync    ( no_csync   ),
    // YPbPr always uses composite sync
    .ypbpr       ( ypbpr      ),
    // Rotate OSD [0] - rotate [1] - left or right
    .rotate      ( 2'b00      ),
    // composite-like blending
    .blend       ( 1'b0       ),

    // video in
    .R           ( {4{pixel}} ),
    .G           ( {4{pixel}} ),
    .B           ( {4{pixel}} ),

    .HSync       ( HSync      ),
    .VSync       ( VSync      ),

    // MiST video output signals
    .VGA_R       ( VGA_R      ),
    .VGA_G       ( VGA_G      ),
    .VGA_B       ( VGA_B      ),
    .VGA_VS      ( vs         ),
    .VGA_HS      ( hs         )
);

// Use different alignment of csync @15kHz
wire   cs = ~(~HSync | ~VSync);
assign VGA_HS = (~no_csync & scandoubler_disable & ~ypbpr) ? cs : hs;
assign VGA_VS = (~no_csync & scandoubler_disable & ~ypbpr) ? 1'b1 : vs;


//////////////////   DISK   //////////////////
assign LED = LED_RED;
 
//-------------------------------------------------------------------------------
//  Ondra SD
//
//
wire OndraSD_signal_led;
wire OndraSD_rxd;
wire OndraSD_txd;
wire SD_MISO;
wire SD_MOSI;
wire SD_SCK;
wire SD_CS;

//assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
//
OndraSD #(.sysclk_frequency(27000000)) OndraSD // 50MHz
(
   //.clk(clk_50M),
   .clk(CLOCK_27[0]),
   .reset_in(~reset),
   .enter_key(kbd_enter),
   .signal_led(OndraSD_signal_led),

   // SPI signals
   .spi_miso(SD_MISO),
   .spi_mosi(SD_MOSI),
   .spi_clk(SD_SCK),
   .spi_cs(SD_CS),
   
   // UART
   .rxd(OndraSD_rxd),// | ~st_ondra_sd),
   .txd(OndraSD_txd)
); 
 
//assign LED_RED = ~SD_CS;

 

endmodule
