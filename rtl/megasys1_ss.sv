// Per-CPU save-state helper: forced level 7 interrupt, handler/vector override and stack pointer capture.
module megasys1_ss_cpu (
    input             clk,
    input             reset,

    input      [23:1] a,
    input             as_n,
    input             rw,
    input             uds_n,
    input             lds_n,
    input       [2:0] fc,
    input      [15:0] dout,

    input             start_save,
    input             start_restore,
    input             drop_irq,
    input             clear,
    input      [31:0] ssp_in,

    output reg        irq,
    output reg        ovr,
    output            hit,
    output reg [15:0] hit_data,
    output reg        parked,
    output reg [31:0] ssp
);

wire ds     = ~( uds_n & lds_n );
wire in_hdl = ( a[23:5] == 19'h7f800 );   // handler, 0xff0000-0xff001f
wire in_rst = ( a[23:3] == 21'd0 );       // reset vectors, 0x000000-0x000007
wire in_vec = ( a[23:2] == 22'h1f );      // level 7 autovector, 0x00007c-0x00007f

assign hit = ovr & ( in_hdl | in_rst | in_vec );

// handler: push d0-a6 and usp, store ssp at 0xff0000; restart point 0xff0008 pops them and returns
always @* begin
    case ( a[4:1] )
        4'h0: hit_data = 16'h48e7;   // movem.l d0-d7/a0-a6,-(a7)
        4'h1: hit_data = 16'hfffe;
        4'h2: hit_data = 16'h4e6e;   // move.l usp,a6
        4'h3: hit_data = 16'h2f0e;   // move.l a6,-(a7)
        4'h4: hit_data = 16'h4df9;   // lea $ff0000,a6
        4'h5: hit_data = 16'h00ff;
        4'h6: hit_data = 16'h0000;
        4'h7: hit_data = 16'h2c8f;   // move.l a7,(a6)
        4'h8: hit_data = 16'h2c5f;   // move.l (a7)+,a6
        4'h9: hit_data = 16'h4e66;   // move.l a6,usp
        4'ha: hit_data = 16'h4cdf;   // movem.l (a7)+,d0-d7/a0-a6
        4'hb: hit_data = 16'h7fff;
        4'hc: hit_data = 16'h4e73;   // rte
        default: hit_data = 16'h0000;
    endcase
    if ( in_rst ) begin
        case ( a[2:1] )
            2'd0: hit_data = ssp_in[31:16];
            2'd1: hit_data = ssp_in[15:0];
            2'd2: hit_data = 16'h00ff;
            2'd3: hit_data = 16'h0008;
        endcase
    end else if ( in_vec ) begin
        hit_data = a[1] ? 16'h0000 : 16'h00ff;
    end
end

always @(posedge clk) begin
    if ( reset ) begin
        irq    <= 0;
        ovr    <= 0;
        parked <= 0;
    end else begin
        if ( start_save )    begin irq <= 1; parked <= 0; end
        if ( start_restore ) begin ovr <= 1; parked <= 0; end
        if ( irq && !as_n && fc == 3'b111 && a[3:1] == 3'b111 ) begin
            irq <= 0;
            ovr <= 1;
        end
        if ( ovr && !as_n && !rw && ds && in_hdl ) begin
            if ( a[4:1] == 4'd0 ) ssp[31:16] <= dout;
            if ( a[4:1] == 4'd1 ) begin ssp[15:0] <= dout; parked <= 1; end
        end
        if ( ovr && !as_n && rw && ds && fc[1:0] == 2'b10 && !( in_hdl | in_rst | in_vec ) ) ovr <= 0;
        if ( drop_irq ) irq <= 0;
        if ( clear ) begin irq <= 0; ovr <= 0; end
    end
end

endmodule


// Save-state sequencer: parks both CPUs at a fixed raster point, streams the state, resumes at the same raster point.
module megasys1_ss_ctrl (
    input             clk,
    input             reset,
    input             do_save,
    input             do_restore,

    input       [8:0] vc,
    input       [8:0] hc,
    input       [7:0] c6,
    input             vbl,
    input             sprite_idle,
    input             s_in_reset,

    input             parked_p,
    input             parked_s,
    input             ovr_p,
    input             ovr_s,
    input      [31:0] ssp_p,
    input      [31:0] ssp_s,

    output reg        start_save_p,
    output reg        start_save_s,
    output reg        start_restore_p,
    output reg        start_restore_s,
    output reg        drop_irq,
    output reg        clear_ovr,
    output reg [31:0] ssp_in_p,
    output reg [31:0] ssp_in_s,

    output reg        halt_p,
    output reg        halt_s,
    output reg        halt_snd,
    output reg        halt_vid,
    output reg        exec_p,
    output reg        exec_s,
    output reg        rst_p,
    output reg        rst_s,
    output reg        oki_rst,

    output reg        opm_replay,
    input             opm_busy,

    output reg        ss_write,
    output reg        ss_read,
    input             ss_busy,

    input             hdr_done,
    input             hdr_ok,
    input             chunks_ok,
    input             wrote_any,
    output reg        hdr_abort,
    output reg        save_ok,
    output            lock,
    output            bad,
    output            dry,

    output            active,

    ssbus_if.slave    ssbus
);

localparam [15:0] MAGIC = 16'h4d31;

typedef enum logic [4:0] {
    IDLE, S_WAIT_R, S_PARK, S_STREAM, S_STREAM_WAIT, S_RESUME, S_EXIT,
    R_WAIT_R, R_HDR, R_DRY, R_WAIT_R2, R_STREAM, R_STREAM_WAIT, R_OPM, R_OPM_WAIT, R_RESET, R_RUN, R_RESUME, R_BAD
} state_t;

state_t     st = IDLE;
reg  [21:0] timer;
reg   [8:0] rf_vc;
reg   [8:0] rf_hc;
reg   [7:0] rf_c6;
reg         s_skip;
reg         magic_ok;
reg  [15:0] rd;

wire at_r  = sprite_idle && !vbl && vc < 9'd208;
wire at_rf = vc == rf_vc && hc == rf_hc && c6 == rf_c6;

// saved raster point reachable; stack pointers even, in work RAM (A19-A0 decode), with room for the 70 bytes the restart code pops
wire ssp_ok_p = !ssp_in_p[0] && ssp_in_p[19:16] == 4'hf && ssp_in_p[15:0] <= 16'hffba;
wire ssp_ok_s = s_skip || ( !ssp_in_s[0] && ssp_in_s[19:17] == 3'h7 && ssp_in_s[16:0] <= 17'h1ffba );
wire glob_ok  = magic_ok && rf_vc <= 9'd262 && rf_hc <= 9'd383 && rf_c6 <= 8'd11 && ssp_ok_p && ssp_ok_s;

assign active = ( st != IDLE );
// a stream is moving state between DDR and the machine: a core reset waits for it
assign lock   = ( st == S_STREAM ) || ( st == S_STREAM_WAIT ) || ( st == R_STREAM ) || ( st == R_STREAM_WAIT );
assign bad    = ( st == R_BAD );
// check pass: the slot is streamed with writes to the machine held back
assign dry    = ( st == R_HDR ) || ( st == R_DRY );

always @* begin
    halt_p   = 0;
    halt_s   = 0;
    halt_snd = 0;
    halt_vid = 0;
    exec_p   = 0;
    exec_s   = 0;
    rst_p    = 0;
    rst_s    = 0;
    oki_rst  = 0;
    case ( st )
        S_PARK: begin
            halt_p = parked_p;
            halt_s = parked_s | s_skip;
            exec_p = ~parked_p;
            exec_s = ~parked_s & ~s_skip;
        end
        S_STREAM, S_STREAM_WAIT, S_RESUME, R_STREAM, R_STREAM_WAIT, R_OPM, R_OPM_WAIT, R_RESUME, R_BAD: begin
            halt_p   = 1;
            halt_s   = 1;
            halt_snd = 1;
            halt_vid = 1;
            oki_rst  = ( st == R_OPM ) || ( st == R_OPM_WAIT );
        end
        R_RESET: begin
            halt_snd = 1;
            halt_vid = 1;
            rst_p    = 1;
            rst_s    = ~s_skip;
            exec_p   = 1;
            exec_s   = ~s_skip;
        end
        R_RUN: begin
            halt_p   = parked_p;
            halt_s   = parked_s | s_skip;
            halt_snd = 1;
            halt_vid = 1;
            exec_p   = ~parked_p;
            exec_s   = ~parked_s & ~s_skip;
        end
        S_EXIT: begin
            exec_p = ovr_p;
            exec_s = ovr_s;
        end
        default: ;
    endcase
end

always @* begin
    case ( ssbus.addr[3:0] )
        4'd0:    rd = ssp_p[31:16];
        4'd1:    rd = ssp_p[15:0];
        4'd2:    rd = ssp_s[31:16];
        4'd3:    rd = ssp_s[15:0];
        4'd4:    rd = { 7'd0, rf_vc };
        4'd5:    rd = { 7'd0, rf_hc };
        4'd6:    rd = { 8'd0, rf_c6 };
        4'd7:    rd = { 15'd0, s_skip };
        default: rd = MAGIC;
    endcase
end

always @(posedge clk) begin
    start_save_p    <= 0;
    start_save_s    <= 0;
    start_restore_p <= 0;
    start_restore_s <= 0;
    drop_irq        <= 0;
    clear_ovr       <= 0;
    opm_replay      <= 0;
    ss_write        <= 0;
    ss_read         <= 0;
    hdr_abort       <= 0;
    save_ok         <= 0;

    // global chunk: stack pointers, raster point, sound CPU reset flag, layout tag
    ssbus.setup(SSIDX_GLOBAL, 32'd9, 1);
    if ( ssbus.access(SSIDX_GLOBAL) ) begin
        if ( ssbus.read ) begin
            ssbus.read_response(SSIDX_GLOBAL, { 48'd0, rd });
        end else if ( ssbus.write ) begin
            case ( ssbus.addr[3:0] )
                4'd0: ssp_in_p[31:16] <= ssbus.data[15:0];
                4'd1: ssp_in_p[15:0]  <= ssbus.data[15:0];
                4'd2: ssp_in_s[31:16] <= ssbus.data[15:0];
                4'd3: ssp_in_s[15:0]  <= ssbus.data[15:0];
                4'd4: rf_vc           <= ssbus.data[8:0];
                4'd5: rf_hc           <= ssbus.data[8:0];
                4'd6: rf_c6           <= ssbus.data[7:0];
                4'd7: s_skip          <= ssbus.data[0];
                4'd8: magic_ok        <= ( ssbus.data[15:0] == MAGIC );
                default: ;
            endcase
            ssbus.write_ack(SSIDX_GLOBAL);
        end
    end

    if ( reset ) begin
        st <= IDLE;
    end else begin
        case ( st )
            IDLE: begin
                timer <= 0;
                if ( do_save )         st <= S_WAIT_R;
                else if ( do_restore ) st <= R_WAIT_R;
            end

            S_WAIT_R: if ( at_r ) begin
                s_skip       <= s_in_reset;
                start_save_p <= 1;
                start_save_s <= ~s_in_reset;
                timer        <= 0;
                st           <= S_PARK;
            end

            S_PARK: begin
                timer <= timer + 1'd1;
                if ( parked_p && ( parked_s || s_skip ) ) begin
                    rf_vc    <= vc;
                    rf_hc    <= hc;
                    rf_c6    <= c6;
                    ss_write <= 1;
                    st       <= S_STREAM;
                end else if ( &timer ) begin
                    drop_irq <= 1;
                    timer    <= 0;
                    st       <= S_EXIT;
                end
            end

            S_STREAM:      if ( ss_busy )  st <= S_STREAM_WAIT;
            S_STREAM_WAIT: if ( !ss_busy ) begin save_ok <= 1; st <= S_RESUME; end
            S_RESUME:      if ( at_rf ) begin timer <= 0; st <= S_EXIT; end

            S_EXIT: begin
                timer <= timer + 1'd1;
                if ( !ovr_p && !ovr_s ) begin
                    st <= IDLE;
                end else if ( &timer ) begin
                    clear_ovr <= 1;
                    st        <= IDLE;
                end
            end

            R_WAIT_R: if ( at_r ) begin
                magic_ok <= 0;
                ss_read  <= 1;
                timer    <= 0;
                st       <= R_HDR;
            end

            // check pass, nothing halted: header length, then every chunk whole and the global values in range
            R_HDR: begin
                timer <= timer + 1'd1;
                if ( hdr_done && hdr_ok ) begin
                    timer <= 0;
                    st    <= R_DRY;
                end else if ( hdr_done || &timer ) begin
                    hdr_abort <= 1;
                    timer     <= 0;
                    st        <= S_EXIT;
                end
            end

            R_DRY: begin
                timer <= timer + 1'd1;
                if ( !ss_busy && chunks_ok && glob_ok ) begin
                    st <= R_WAIT_R2;
                end else if ( !ss_busy || &timer ) begin
                    hdr_abort <= 1;
                    timer     <= 0;
                    st        <= S_EXIT;
                end
            end

            R_WAIT_R2: if ( at_r ) begin
                magic_ok <= 0;
                ss_read  <= 1;
                st       <= R_STREAM;
            end

            R_STREAM: if ( ss_busy ) st <= R_STREAM_WAIT;

            // every chunk must arrive whole, or the partly loaded machine is reset instead of resumed
            R_STREAM_WAIT: if ( !ss_busy ) begin
                if ( magic_ok && chunks_ok ) begin
                    opm_replay <= 1;
                    st         <= R_OPM;
                end else if ( !wrote_any ) begin
                    timer <= 0;
                    st    <= S_EXIT;
                end else begin
                    st    <= R_BAD;
                end
            end

            R_BAD: ;

            R_OPM:      if ( opm_busy ) st <= R_OPM_WAIT;
            R_OPM_WAIT: if ( !opm_busy ) begin
                start_restore_p <= 1;
                start_restore_s <= ~s_skip;
                timer           <= 0;
                st              <= R_RESET;
            end

            R_RESET: begin
                timer <= timer + 1'd1;
                if ( timer == 22'd4095 ) begin
                    timer <= 0;
                    st    <= R_RUN;
                end
            end

            // CPUs run the restart code up to the stack pointer store, where the save parked them
            R_RUN: begin
                timer <= timer + 1'd1;
                if ( parked_p && ( parked_s || s_skip ) ) begin
                    timer <= 0;
                    st    <= R_RESUME;
                end else if ( &timer ) begin
                    timer <= 0;
                    st    <= S_EXIT;
                end
            end

            R_RESUME: begin
                timer <= timer + 1'd1;
                if ( at_rf ) begin
                    timer <= 0;
                    st    <= S_EXIT;
                end else if ( &timer ) begin
                    st    <= R_BAD;
                end
            end

            default: st <= IDLE;
        endcase
    end
end

endmodule


// Save-state access to one RAM port: stream address/data/write while selected, read data returned one cycle later.
module megasys1_ss_ram #(parameter AW = 10, parameter SS_IDX = -1) (
    input              clk,
    ssbus_if.slave     ssbus,
    output             sel,
    output    [AW-1:0] addr,
    output      [15:0] wdata,
    output             we,
    input       [15:0] rdata
);

wire [31:0] SIZE = 2**AW;

assign sel   = ssbus.access(SS_IDX);
assign addr  = ssbus.addr[AW-1:0];
assign wdata = ssbus.data[15:0];
assign we    = sel & ssbus.write;

reg read_delay = 0;

always @(posedge clk) begin
    ssbus.setup(SS_IDX, SIZE, 1);
    if ( sel ) begin
        if ( ssbus.write ) begin
            ssbus.write_ack(SS_IDX);
        end else if ( ssbus.read ) begin
            if ( read_delay ) ssbus.read_response(SS_IDX, { 48'd0, rdata });
            read_delay <= 1;
        end
    end else begin
        read_delay <= 0;
    end
end

endmodule


// YM2151 register shadow: records register writes, saves them, and replays them into the chip after a restore.
module megasys1_opm_shadow #(parameter SS_IDX_REGS = -1, parameter SS_IDX_MISC = -1) (
    input            clk,
    input            clr,
    input            wr,
    input            a0,
    input      [7:0] din,

    input            replay,
    output           busy,
    output reg       ym_wr,
    output reg       ym_a0,
    output reg [7:0] ym_din,
    output reg       ym_ic = 1'b0,
    input            ym_busy,

    ssbus_if.slave   ssb_regs,
    ssbus_if.slave   ssb_misc
);

reg   [7:0] regs [0:255];
reg   [7:0] cur_addr = 0;
reg  [31:0] keyon = 0;
reg   [6:0] pmd = 0;
reg   [6:0] amd = 0;
reg [255:0] valid = 0;

wire [309:0] misc_out;
wire         misc_wr;

auto_save_adaptor #(.N_BITS(310), .SS_IDX(SS_IDX_MISC)) misc_ss (
    .clk,
    .ssbus(ssb_misc),
    .bits_in({ valid, amd, pmd, keyon, cur_addr }),
    .bits_out(misc_out),
    .bits_wr(misc_wr)
);

// register file: written by the chip writes or the stream, read by the stream or the replay
wire       ss_sel = ssb_regs.access(SS_IDX_REGS);
reg  [7:0] rp_addr;
reg  [7:0] rd_q;
wire [7:0] wa = ss_sel ? ssb_regs.addr[7:0] : cur_addr;
wire       we = ss_sel ? ssb_regs.write : ( wr & a0 );
wire [7:0] wd = ss_sel ? ssb_regs.data[7:0] : din;

always @(posedge clk) begin
    if ( we ) regs[wa] <= wd;
    rd_q <= regs[ss_sel ? ssb_regs.addr[7:0] : rp_addr];
end

reg rdly = 0;
always @(posedge clk) begin
    ssb_regs.setup(SS_IDX_REGS, 32'd256, 1);
    if ( ss_sel ) begin
        if ( ssb_regs.write ) begin
            ssb_regs.write_ack(SS_IDX_REGS);
        end else if ( ssb_regs.read ) begin
            if ( rdly ) ssb_regs.read_response(SS_IDX_REGS, { 56'd0, rd_q });
            rdly <= 1;
        end
    end else begin
        rdly <= 0;
    end
end

// address latch, written-register map, per-channel key-on and PMD/AMD (both share register 0x19)
always @(posedge clk) begin
    if ( clr ) begin
        cur_addr <= 0;
        keyon    <= 0;
        pmd      <= 0;
        amd      <= 0;
        valid    <= 0;
    end else if ( misc_wr ) begin
        { valid, amd, pmd, keyon, cur_addr } <= misc_out;
    end else if ( wr ) begin
        if ( !a0 ) begin
            cur_addr <= din;
        end else begin
            valid[cur_addr] <= 1'b1;
            if ( cur_addr == 8'h08 ) keyon[ { din[2:0], 2'b00 } +: 4 ] <= din[6:3];
            if ( cur_addr == 8'h19 ) begin
                if ( din[7] ) pmd <= din[6:0];
                else          amd <= din[6:0];
            end
        end
    end
end

// replay: chip reset, then written registers 0x20-0xff, globals, timers, AMD, PMD, key-on per channel, 0x14, address latch
reg  [7:0] step;
reg  [7:0] s_reg;
reg        s_ram;
reg  [7:0] s_fixed;
reg        s_valid;
reg        s_addr_only;
wire [7:0] s_ch = step - 8'd233;

always @* begin
    s_ram       = 1;
    s_fixed     = 8'd0;
    s_addr_only = 0;
    s_reg       = 8'h00;
    if ( step < 8'd224 ) begin
        s_reg = 8'h20 + step;
    end else if ( step < 8'd231 ) begin
        case ( step - 8'd224 )
            8'd0:    s_reg = 8'h01;
            8'd1:    s_reg = 8'h0f;
            8'd2:    s_reg = 8'h18;
            8'd3:    s_reg = 8'h1b;
            8'd4:    s_reg = 8'h10;
            8'd5:    s_reg = 8'h11;
            default: s_reg = 8'h12;
        endcase
    end else if ( step < 8'd233 ) begin
        s_reg   = 8'h19;
        s_ram   = 0;
        s_fixed = ( step == 8'd231 ) ? { 1'b0, amd } : { 1'b1, pmd };
    end else if ( step < 8'd241 ) begin
        s_reg   = 8'h08;
        s_ram   = 0;
        s_fixed = { 1'b0, keyon[ { s_ch[2:0], 2'b00 } +: 4 ], s_ch[2:0] };
    end else if ( step == 8'd241 ) begin
        // timer control last, so the timers restart as close to the resume as possible
        s_reg = 8'h14;
    end else begin
        s_reg       = cur_addr;
        s_addr_only = 1;
    end
    s_valid = ( ( step >= 8'd233 && step < 8'd241 ) || step > 8'd241 ) ? 1'b1 : valid[s_reg];
end

typedef enum logic [3:0] { P_IDLE, P_IC, P_IC_GAP, P_PREP, P_READ, P_DATA, P_AW, P_AW_GAP, P_DW, P_DW_WAIT } pstate_t;
pstate_t   ps = P_IDLE;
reg [11:0] cnt;
reg  [7:0] data_r;

assign busy = ( ps != P_IDLE );

always @(posedge clk) begin
    case ( ps )
        P_IDLE: begin
            ym_wr <= 0;
            ym_ic <= 0;
            if ( replay ) begin
                step  <= 0;
                cnt   <= 0;
                ym_ic <= 1;
                ps    <= P_IC;
            end
        end
        // chip reset first, so registers the saved state never wrote are back at their reset value 0
        P_IC: begin
            cnt <= cnt + 1'd1;
            if ( cnt == 12'd4095 ) begin
                ym_ic <= 0;
                cnt   <= 0;
                ps    <= P_IC_GAP;
            end
        end
        P_IC_GAP: begin
            cnt <= cnt + 1'd1;
            if ( cnt == 12'd511 ) ps <= P_PREP;
        end
        P_PREP: begin
            rp_addr <= s_reg;
            if ( step > 8'd242 )  ps <= P_IDLE;
            else if ( !s_valid )  step <= step + 1'd1;
            else                  ps <= P_READ;
        end
        P_READ: ps <= P_DATA;
        P_DATA: begin
            data_r <= s_ram ? rd_q : s_fixed;
            ym_a0  <= 0;
            ym_din <= s_reg;
            ym_wr  <= 1;
            cnt    <= 0;
            ps     <= P_AW;
        end
        P_AW: begin
            cnt <= cnt + 1'd1;
            if ( cnt == 11'd47 ) begin
                ym_wr <= 0;
                cnt   <= 0;
                ps    <= P_AW_GAP;
            end
        end
        P_AW_GAP: begin
            cnt <= cnt + 1'd1;
            if ( cnt == 11'd47 ) begin
                if ( s_addr_only ) begin
                    step <= step + 1'd1;
                    ps   <= P_PREP;
                end else begin
                    ym_a0  <= 1;
                    ym_din <= data_r;
                    ym_wr  <= 1;
                    cnt    <= 0;
                    ps     <= P_DW;
                end
            end
        end
        P_DW: begin
            cnt <= cnt + 1'd1;
            if ( cnt == 11'd47 ) begin
                ym_wr <= 0;
                cnt   <= 0;
                ps    <= P_DW_WAIT;
            end
        end
        P_DW_WAIT: begin
            if ( cnt != 11'h7ff ) cnt <= cnt + 1'd1;
            if ( cnt >= 11'd1535 && !ym_busy ) begin
                step <= step + 1'd1;
                ps   <= P_PREP;
            end
        end
        default: ps <= P_IDLE;
    endcase
end

endmodule


// Buffers screen_rotate's single-beat DDRAM writes and replays them as a ddr_if master that honours busy.
module megasys1_rotate_ddr (
    input            clk,
    input     [28:0] addr,
    input     [63:0] din,
    input      [7:0] be,
    input            we,
    ddr_if.to_host   ddr
);

reg [68:0] fifo [0:1023];
reg  [9:0] wp = 0;
reg  [9:0] rp = 0;
reg [68:0] q;
reg  [1:0] st = 0;

wire fifo_we = we && ( wp + 1'd1 ) != rp;

assign ddr.read     = 0;
assign ddr.burstcnt = 8'd1;

always @(posedge clk) begin
    if ( fifo_we ) fifo[wp] <= { addr, be, din[31:0] };
    q <= fifo[rp];
end

always @(posedge clk) begin
    if ( fifo_we ) wp <= wp + 1'd1;

    case ( st )
        2'd0: if ( rp != wp ) begin
                  ddr.acquire <= 1;
                  st          <= 2'd1;
              end else begin
                  ddr.acquire <= 0;
              end
        2'd1: st <= 2'd2;
        2'd2: begin
                  ddr.addr       <= { q[68:40], 3'b000 };
                  ddr.byteenable <= q[39:32];
                  ddr.wdata      <= { q[31:0], q[31:0] };
                  ddr.write      <= 1;
                  st             <= 2'd3;
              end
        2'd3: if ( !ddr.busy ) begin
                  ddr.write <= 0;
                  rp        <= rp + 1'd1;
                  st        <= 2'd0;
              end
    endcase
end

endmodule


// DDRAM arbiter: save-state stream (a) and rotation FIFO (b) alternate, one posted request at a time.
module megasys1_ddr_arb (
    input            clk,
    input            clr,
    ddr_if.to_host   x,
    ddr_if.from_host a,
    ddr_if.from_host b
);

reg        pend   = 0;
reg        last_b = 0;
reg        p_rd, p_wr;
reg [31:0] p_addr;
reg [63:0] p_wdata;
reg  [7:0] p_be, p_burst;
reg  [1:0] outst  = 0;
reg  [1:0] drop   = 0;

wire a_req   = a.read | a.write;
wire b_req   = b.read | b.write;
wire grant_b = !pend & b_req & ( !a_req | !last_b );
wire grant_a = !pend & a_req & !grant_b;
wire accept  = pend & !x.busy;
wire rd_acc  = accept & p_rd;
wire [1:0] outst_next = outst + rd_acc - x.rdata_ready;

assign a.busy        = a_req & !grant_a;
assign b.busy        = b_req & !grant_b;
assign a.rdata       = x.rdata;
assign b.rdata       = x.rdata;
assign a.rdata_ready = x.rdata_ready & ( drop == 0 );
assign b.rdata_ready = 0;

assign x.read       = pend & p_rd;
assign x.write      = pend & p_wr;
assign x.addr       = p_addr;
assign x.wdata      = p_wdata;
assign x.byteenable = p_be;
assign x.burstcnt   = p_burst;
assign x.acquire    = a.acquire | b.acquire;

always @(posedge clk) begin
    outst <= outst_next;
    if ( clr )                                drop <= outst_next;
    else if ( x.rdata_ready && drop != 0 )    drop <= drop - 1'd1;

    if ( grant_a | grant_b ) begin
        pend    <= 1;
        last_b  <= grant_b;
        p_rd    <= grant_b ? b.read       : a.read;
        p_wr    <= grant_b ? b.write      : a.write;
        p_addr  <= grant_b ? b.addr       : a.addr;
        p_wdata <= grant_b ? b.wdata      : a.wdata;
        p_be    <= grant_b ? b.byteenable : a.byteenable;
        p_burst <= grant_b ? b.burstcnt   : a.burstcnt;
    end else if ( accept ) begin
        pend    <= 0;
    end
end

endmodule


// Restore checks: header length against this layout, and per chunk words written against the queried count.
module megasys1_ss_check #(parameter COUNT = 12, parameter [29:0] WORDS = 30'd80586) (
    input             clk,
    input             start,
    input             rd_valid,
    input      [63:0] rd_data,
    input       [7:0] sel,
    input             write,
    input             query,
    input             ack,
    input      [63:0] ack_data,
    output reg        hdr_done,
    output reg        hdr_ok,
    output            chunks_ok,
    output reg        wrote_any
);

reg             first = 0;
reg  [31:0]     exp_cnt [0:COUNT-1];
reg  [31:0]     got_cnt [0:COUNT-1];
reg [COUNT-1:0] seen;
reg [COUNT-1:0] match;
integer         i;

always @* for ( i = 0; i < COUNT; i = i + 1 ) match[i] = seen[i] && ( got_cnt[i] == exp_cnt[i] );
assign chunks_ok = &match;

always @(posedge clk) begin
    if ( start ) begin
        first     <= 1;
        hdr_done  <= 0;
        hdr_ok    <= 0;
        wrote_any <= 0;
        seen      <= 0;
        for ( i = 0; i < COUNT; i = i + 1 ) got_cnt[i] <= 0;
    end else begin
        if ( first && rd_valid ) begin
            first    <= 0;
            hdr_done <= 1;
            hdr_ok   <= ( rd_data[61:32] == WORDS );
        end
        if ( ack && query && sel < COUNT ) begin
            exp_cnt[sel] <= ack_data[31:0];
            seen[sel]    <= 1;
        end
        if ( ack && write && !query && sel < COUNT ) begin
            got_cnt[sel] <= got_cnt[sel] + 1'd1;
            wrote_any    <= 1;
        end
    end
end

endmodule


// Check pass gate: while dry, writes to the blocks (except the global chunk) are held back and acknowledged here.
module megasys1_ss_dry #(parameter [7:0] KEEP = 8'd0) (
    input              clk,
    input              dry,
    ssbus_if.slave     up,
    ssbus_if.master    dn
);

reg  fake = 0;
wire hold = dry & up.write & ~up.query & ( up.select != KEEP );

assign dn.data     = up.data;
assign dn.addr     = up.addr;
assign dn.select   = up.select;
assign dn.read     = up.read;
assign dn.query    = up.query;
assign dn.write    = up.write & ~hold;
assign up.data_out = dn.data_out;
assign up.ack      = dn.ack | fake;

always @(posedge clk) fake <= hold & ~fake;

endmodule
