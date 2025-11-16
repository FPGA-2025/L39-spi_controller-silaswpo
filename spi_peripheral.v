/*
============================================================
| TABELA DE MODOS SPI (CPOL/CPHA)                          |
============================================================
| Modo | CPOL | CPHA | Clock Inativo  | Leitura (MOSI)    | Escrita (MISO)     |
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
    0 = Dados são amostrados na primeira borda, deslocados na segunda
    1 = Dados são deslocados na primeira borda, amostrados na segunda

Temporização:
- Leitura (MOSI): quando o escravo lê o dado enviado pelo mestre
- Escrita (MISO): quando o escravo coloca o próximo bit para ser lido

Este módulo ajusta seu comportamento automaticamente com base no SPI_MODE.
============================================================
*/


module SPI_Peripheral #(
    parameter SPI_BITS_PER_WORD = 8,
    parameter SPI_MODE          = 0  // 0: CPOL=0, CPHA=0; 1: CPOL=0, CPHA=1; 2: CPOL=1, CPHA=0; 3: CPOL=1, CPHA=1
)(
    input  wire clk,
    input  wire rst_n,

    input  wire sck,
    input  wire cs,
    input  wire mosi,
    output wire miso,

    input  wire data_in_valid,
    output reg  data_out_valid,
    output reg  busy,

    input  wire [SPI_BITS_PER_WORD-1:0] data_in,
    output reg  [SPI_BITS_PER_WORD-1:0] data_out
);

    localparam BIT_COUNT_WIDTH = $clog2(SPI_BITS_PER_WORD);

    // Mode decoding
    localparam CPOL = (SPI_MODE == 2 || SPI_MODE == 3);
    localparam CPHA = (SPI_MODE == 1 || SPI_MODE == 3);

    reg [SPI_BITS_PER_WORD - 1:0] data_in_reg, data_to_send;
    reg [BIT_COUNT_WIDTH   - 1:0] bit_count;
    reg [2:0] sck_sync, cs_sync;
    reg [1:0] mosi_sync;
    reg sync_init;

    wire rising_edge, falling_edge;
    wire sampling_edge, shifting_edge;
    wire cs_active, start_message, end_message;

    // Clock edges
    assign rising_edge   = ~sck_sync[2] & sck_sync[1];
    assign falling_edge  = ~sck_sync[1] & sck_sync[2];

    // SPI mode edge selection
    assign sampling_edge = (CPHA == 0) ? (CPOL ? falling_edge : rising_edge)
                                       : (CPOL ? rising_edge  : falling_edge);

    assign shifting_edge = (CPHA == 0) ? (CPOL ? rising_edge  : falling_edge)
                                       : (CPOL ? falling_edge : rising_edge);

    assign cs_active     = ~cs_sync[1];
    assign start_message = ~cs_sync[1] & cs_sync[2];
    assign end_message   = ~cs_sync[2] & cs_sync[1];
    assign miso          = data_to_send[SPI_BITS_PER_WORD-1];

    // Synchronize external signals
    always @(posedge clk) begin
        if (!rst_n) begin
            sck_sync  <= 3'b000;
            cs_sync   <= 3'b000;
            mosi_sync <= 2'b00;
        end else begin
            sck_sync  <= {sck_sync[1:0], sck};
            cs_sync   <= {cs_sync[1:0], cs};
            mosi_sync <= {mosi_sync[0], mosi};
        end
    end

    // Bit counter and MOSI capture
    always @(posedge clk) begin
        if (!cs_active) begin
            bit_count <= {BIT_COUNT_WIDTH{1'b0}};
        end else begin
            if (sampling_edge) begin
                bit_count <= bit_count + 1'b1;
                data_out  <= {data_out[SPI_BITS_PER_WORD-2:0], mosi_sync[1]};
            end
        end

        if (start_message) begin
            busy <= 1'b0;
        end

        if (end_message) begin
            busy <= 1'b1;
        end
    end

    // Data valid flag
    always @(posedge clk) begin
        if (!rst_n) begin
            data_out_valid <= 1'b0;
        end else begin
            data_out_valid <= cs_active && sampling_edge &&
                              (bit_count == SPI_BITS_PER_WORD - 1);
        end
    end

    // Latch input data
    always @(posedge clk) begin
        if (!rst_n) begin
            data_in_reg <= {SPI_BITS_PER_WORD{1'b0}};
        end else begin
            if (data_in_valid) begin
                data_in_reg <= data_in;
            end
        end
    end

    // Shift out data on MISO
    always @(posedge clk) begin
        if (!rst_n) begin
            sync_init <= 1'b0;
        end else if (cs_active) begin
            if (start_message) begin
                data_to_send <= data_in_reg;
                sync_init    <= CPHA; // if CPHA=0, we already put MSB on MISO
            end else if (shifting_edge) begin
                if(sync_init) begin
                    // first shifting edge after CS active
                    sync_init <= 1'b0;
                end else begin
                    data_to_send <= {data_to_send[SPI_BITS_PER_WORD-2:0], 1'b0};
                end
            end
        end
    end

endmodule
