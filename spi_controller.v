/*
============================================================
| TABELA DE MODOS SPI (CPOL/CPHA)                          |
============================================================
| Modo | CPOL | CPHA | Clock Inativo  | Leitura (MISO)    | Escrita (MOSI)     |
|------|------|------|----------------|-------------------|--------------------|
|  0   |  0   |  0   |   Nível baixo  | Borda de subida   | Borda de descida   |
|  1   |  0   |  1   |   Nível baixo  | Borda de descida  | Borda de subida    |
|  2   |  1   |  0   |   Nível alto   | Borda de descida  | Borda de subida    |
|  3   |  1   |  1   |   Nível alto   | Borda de subida   | Borda de descida   |
------------------------------------------------------------

Definições:
- CPOL (Polaridade do Clock):
    0 = Clock inativo em nível baixo
    1 = Clock inativo em nível alto

- CPHA (Fase do Clock):
    0 = Dados são amostrados na primeira borda, enviados na segunda
    1 = Dados são enviados na primeira borda, amostrados na segunda

Temporização:
- Leitura (MISO): quando o mestre lê o dado vindo do escravo
- Escrita (MOSI): quando o mestre envia o dado ao escravo

Este módulo ajusta seu comportamento automaticamente com base no SPI_MODE.
============================================================
*/


/*
============================================================
| TABELA DE MODOS SPI (CPOL/CPHA)                          |
============================================================
| Modo | CPOL | CPHA | Clock Inativo | Leitura (MISO) | Escrita (MOSI) |
|------|------|------|---------------|----------------|----------------|
|  0   |  0   |  0   | baixo         | subida         | descida        |
|  1   |  0   |  1   | baixo         | descida        | subida         |
|  2   |  1   |  0   | alto          | descida        | subida         |
|  3   |  1   |  1   | alto          | subida         | descida        |
------------------------------------------------------------
*/

module SPI_Controller #(
    parameter SPI_BITS_PER_WORD = 8,
    parameter SPI_MODE          = 0,          // 0: CPOL=0, CPHA=0; 1: CPOL=0, CPHA=1; 2: CPOL=1, CPHA=0; 3: CPOL=1, CPHA=1
    parameter SPI_CLK_FREQ      = 1_000_000,  // 1MHz
    parameter SYS_CLK_FREQ      = 25_000_000  // 25MHz
) (
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
    // ----------------------------
    // Decodifica modo
    // ----------------------------
    localparam CPOL = (SPI_MODE == 2 || SPI_MODE == 3);
    localparam CPHA = (SPI_MODE == 1 || SPI_MODE == 3);

    // ----------------------------
    // clog2 simples
    // ----------------------------
    function integer clog2;
        input integer value; integer i;
        begin
            clog2 = 0;
            for (i = value-1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    // ----------------------------
    // Divisor para gerar SCK
    // ----------------------------
    localparam integer DIV   = (SYS_CLK_FREQ / (2*SPI_CLK_FREQ)) < 1 ? 1
                               : (SYS_CLK_FREQ / (2*SPI_CLK_FREQ));
    localparam integer DIV_W = clog2(DIV);

    // ----------------------------
    // FSM
    // ----------------------------
    localparam [1:0]
        ST_IDLE     = 2'b00,
        ST_TRANSFER = 2'b01,
        ST_DONE     = 2'b10;

    reg [1:0] state;

    // Contador do divisor de SCK
    reg [DIV_W-1:0] div_cnt;

    // Registradores de TX/RX
    reg [SPI_BITS_PER_WORD-1:0] tx_shift;
    reg [SPI_BITS_PER_WORD-1:0] rx_shift;
    // Contador de bits que AINDA faltam ser amostrados
    reg [clog2(SPI_BITS_PER_WORD):0] bit_cnt;

    // Lógica de borda de SCK (usando valor atual de sck antes de inverter)
    wire edge_is_rise   = (sck == 1'b0);          // se hoje está 0, a borda agora é 0->1 (subida)
    wire sample_is_rise = (CPHA == CPOL);         // mesma expressão da tabela
    wire do_sample_edge =  sample_is_rise ? edge_is_rise : ~edge_is_rise;
    wire do_shift_edge  = ~sample_is_rise ? edge_is_rise : ~edge_is_rise;

    // ----------------------------
    // Máquina de estados principal
    // ----------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            sck            <= CPOL;
            cs             <= 1'b1;
            mosi           <= 1'b0;
            busy           <= 1'b0;
            data_out       <= {SPI_BITS_PER_WORD{1'b0}};
            data_out_valid <= 1'b0;
            tx_shift       <= {SPI_BITS_PER_WORD{1'b0}};
            rx_shift       <= {SPI_BITS_PER_WORD{1'b0}};
            bit_cnt        <= {clog2(SPI_BITS_PER_WORD)+1{1'b0}};
            div_cnt        <= {DIV_W{1'b0}};
        end else begin
            // por padrão, data_out_valid é pulso de 1 ciclo
            data_out_valid <= 1'b0;

            case (state)
            // ---------------- IDLE ----------------
            ST_IDLE: begin
                sck     <= CPOL;             // clock parado no nível inativo
                cs      <= 1'b1;             // sem transação
                busy    <= 1'b0;
                div_cnt <= {DIV_W{1'b0}};    // reseta divisor

                if (data_in_valid) begin
                    // começa nova transação
                    busy     <= 1'b1;
                    cs       <= 1'b0;       // seleciona escravo
                    tx_shift <= data_in;    // carrega dado que vamos transmitir
                    rx_shift <= {SPI_BITS_PER_WORD{1'b0}};
                    bit_cnt  <= SPI_BITS_PER_WORD; // ainda faltam N bits

                    // CPHA=0: já coloca MSB no MOSI antes da 1ª borda de leitura
                    if (CPHA == 1'b0)
                        mosi <= data_in[SPI_BITS_PER_WORD-1];

                    state   <= ST_TRANSFER;
                end
            end

            // ---------------- TRANSFER ----------------
            ST_TRANSFER: begin
                // Gera tick de SCK
                if (div_cnt == DIV-1) begin
                    div_cnt <= {DIV_W{1'b0}};

                    // Nesta borda vamos decidir se é de sample ou de shift
                    // edge_is_rise olha o valor ATUAL de sck (antes de inverter)
                    // do_sample_edge / do_shift_edge decidem conforme modo

                    // 1) Sample de MISO (leitura do escravo)
                    if (do_sample_edge && (bit_cnt != 0)) begin
                        rx_shift <= {rx_shift[SPI_BITS_PER_WORD-2:0], miso};
                        bit_cnt  <= bit_cnt - 1'b1;

                        // se era o último bit que faltava, fecha o byte
                        if (bit_cnt == 1) begin
                            data_out       <= {rx_shift[SPI_BITS_PER_WORD-2:0], miso};
                            data_out_valid <= 1'b1;
                        end
                    end

                    // 2) Escrita em MOSI (shift de TX)
                    if (do_shift_edge && (bit_cnt != 0)) begin
                        if (CPHA == 1'b0) begin
                            // CPHA=0: 1º bit já foi colocado lá no IDLE
                            // aqui só avançamos para os próximos
                            mosi     <= tx_shift[SPI_BITS_PER_WORD-2];
                            tx_shift <= {tx_shift[SPI_BITS_PER_WORD-2:0], 1'b0};
                        end else begin
                            // CPHA=1: 1ª borda de shift já joga o MSB em MOSI
                            mosi     <= tx_shift[SPI_BITS_PER_WORD-1];
                            tx_shift <= {tx_shift[SPI_BITS_PER_WORD-2:0], 1'b0};
                        end
                    end

                    // 3) Inverte SCK (gera a borda física)
                    sck <= ~sck;

                    // 4) Se acabou os bits, vai para DONE
                    if (bit_cnt == 0) begin
                        state <= ST_DONE;
                    end
                end else begin
                    // ainda não é hora da próxima borda de SCK
                    div_cnt <= div_cnt + 1'b1;
                end
            end

            // ---------------- DONE ----------------
            ST_DONE: begin
                sck  <= CPOL;  // volta clock pro nível inativo
                cs   <= 1'b1;  // deseleciona escravo
                busy <= 1'b0;  // transação terminou
                state<= ST_IDLE;
            end

            endcase
        end
    end

endmodule
