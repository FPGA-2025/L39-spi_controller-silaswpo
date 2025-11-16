/*
============================================================
| TABELA DE MODOS SPI (CPOL/CPHA)                          |
============================================================
| Modo | CPOL | CPHA | Clock Inativo | Leitura | Escrita   |
|------|------|------|---------------|---------|-----------|
|  0   |  0   |  0   | baixo         | subida  | descida   |
|  1   |  0   |  1   | baixo         | descida | subida    |
|  2   |  1   |  0   | alto          | descida | subida    |
|  3   |  1   |  1   | alto          | subida  | descida   |
------------------------------------------------------------
*/

module SPI_Controller #(
    parameter SPI_BITS_PER_WORD = 8,
    parameter SPI_MODE          = 0,
    parameter SPI_CLK_FREQ      = 1_000_000,
    parameter SYS_CLK_FREQ      = 25_000_000
)(
    input  wire clk,
    input  wire rst_n,

    output reg  sck,
    output reg  mosi,
    input  wire miso,

    output reg  cs,

    input  wire data_in_valid,
    output reg  data_out_valid,

    input  wire [SPI_BITS_PER_WORD-1:0] data_in,
    output reg  [SPI_BITS_PER_WORD-1:0] data_out,

    output reg  busy
);

    // CPOL e CPHA conforme tabela
    localparam CPOL = (SPI_MODE == 2 || SPI_MODE == 3);
    localparam CPHA = (SPI_MODE == 1 || SPI_MODE == 3);

    // Função para calcular quantos bits o contador precisa
    function integer clog2;
        input integer value; integer i;
        begin
            clog2 = 0;
            for (i = value-1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    // Divisor para gerar o clock SCK desejado
    localparam integer DIV   = (SYS_CLK_FREQ / (2*SPI_CLK_FREQ)) < 1 ? 1
                               : (SYS_CLK_FREQ / (2*SPI_CLK_FREQ));
    localparam integer DIV_W = clog2(DIV);

    // Estados da máquina
    localparam [1:0]
        ST_IDLE     = 2'b00,
        ST_TRANSFER = 2'b01,
        ST_DONE     = 2'b10;

    reg [1:0] state;

    // Contador para dividir a frequência
    reg [DIV_W-1:0] div_cnt;

    // Registradores de envio/recebimento
    reg [SPI_BITS_PER_WORD-1:0] tx_shift;
    reg [SPI_BITS_PER_WORD-1:0] rx_shift;

    // Conta quantos bits faltam enviar
    reg [clog2(SPI_BITS_PER_WORD):0] bit_cnt;

    // Descobre se a borda de hoje é subida ou descida
    wire edge_is_rise   = (sck == 1'b0);

    // Decide em qual borda vamos ler ou escrever
    wire sample_is_rise = (CPHA == CPOL);
    wire do_sample_edge =  sample_is_rise ? edge_is_rise : ~edge_is_rise;
    wire do_shift_edge  = ~sample_is_rise ? edge_is_rise : ~edge_is_rise;

    // ----------------------------------------------------
    // Máquina de estados principal
    // ----------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // valores iniciais
            state          <= ST_IDLE;
            sck            <= CPOL;
            cs             <= 1'b1;
            mosi           <= 1'b0;
            busy           <= 1'b0;
            data_out       <= 0;
            data_out_valid <= 0;
            tx_shift       <= 0;
            rx_shift       <= 0;
            bit_cnt        <= 0;
            div_cnt        <= 0;
        end else begin
            // pulso de 1 ciclo quando terminar
            data_out_valid <= 1'b0;

            case (state)

            // ===========================
            //         IDLE
            // ===========================
            ST_IDLE: begin
                sck     <= CPOL;
                cs      <= 1'b1;
                busy    <= 1'b0;
                div_cnt <= 0;

                // Se recebeu pedido de envio, começa tudo
                if (data_in_valid) begin
                    busy     <= 1'b1;
                    cs       <= 1'b0;
                    tx_shift <= data_in;
                    rx_shift <= 0;
                    bit_cnt  <= SPI_BITS_PER_WORD;

                    // CPHA=0 → já coloca o primeiro bit no MOSI
                    if (CPHA == 1'b0)
                        mosi <= data_in[SPI_BITS_PER_WORD-1];

                    state <= ST_TRANSFER;
                end
            end

            // ===========================
            //       TRANSFERÊNCIA
            // ===========================
            ST_TRANSFER: begin
                // Gera o tick do SCK
                if (div_cnt == DIV-1) begin
                    div_cnt <= 0;

                    // 1) Ler MISO (bit vindo do escravo)
                    if (do_sample_edge && bit_cnt != 0) begin
                        rx_shift <= {rx_shift[SPI_BITS_PER_WORD-2:0], miso};
                        bit_cnt  <= bit_cnt - 1;

                        // último bit chegando
                        if (bit_cnt == 1) begin
                            data_out       <= {rx_shift[SPI_BITS_PER_WORD-2:0], miso};
                            data_out_valid <= 1;
                        end
                    end

                    // 2) Atualiza MOSI (bit que enviamos)
                    if (do_shift_edge && bit_cnt != 0) begin
                        if (CPHA == 1'b0) begin
                            // aqui só avança para os próximos bits
                            mosi     <= tx_shift[SPI_BITS_PER_WORD-2];
                            tx_shift <= {tx_shift[SPI_BITS_PER_WORD-2:0], 1'b0};
                        end else begin
                            // CPHA=1 → joga o MSB logo no shift
                            mosi     <= tx_shift[SPI_BITS_PER_WORD-1];
                            tx_shift <= {tx_shift[SPI_BITS_PER_WORD-2:0], 1'b0};
                        end
                    end

                    // 3) Gera a borda de clock física
                    sck <= ~sck;

                    // Se acabou os bits, muda de estado
                    if (bit_cnt == 0)
                        state <= ST_DONE;

                end else begin
                    div_cnt <= div_cnt + 1;
                end
            end

            // ===========================
            //        DONE
            // ===========================
            ST_DONE: begin
                sck  <= CPOL;
                cs   <= 1'b1;
                busy <= 1'b0;
                state <= ST_IDLE;
            end

            endcase
        end
    end

endmodule
