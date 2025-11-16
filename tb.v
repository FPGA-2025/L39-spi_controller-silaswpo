`timescale 1ns/1ps

module tb();

    reg clk;
    reg rst_n;

    always #1 clk = ~clk;

    integer i;

    reg [7:0] test_data [0:3];

    initial begin
        $dumpfile("saida.vcd");
        $dumpvars(0, tb);

        clk      = 0;
        rst_n    = 0;
        #5 rst_n = 1;

        // Dados de teste
        test_data[0] = 8'hA5;
        test_data[1] = 8'h5A;
        test_data[2] = 8'hFF;
        test_data[3] = 8'h00;

        // Espera 10 ciclos de clock
        repeat (10) @(posedge clk);
        // Envio de bytes
        for (i = 0; i < 4; i = i + 1) begin
            data_in_valid = 1;
            slave_data_in_valid = 1;
            data_in = test_data[i];
            slave_data_in = test_data[i];
            @(posedge clk);
            @(negedge clk);
            data_in_valid = 0;
            slave_data_in_valid = 0;
            
            @(posedge clk);

            wait (busy == 0);

            @(posedge clk);

            if (data_out == test_data[i])
                $display("Test %0d: Controller OK! Received=%h, Expected=%h", i, data_out, test_data[i]);
            else
                $display("Test %0d: Controller ERRO! Received=%h, Expected=%h", i, data_out, test_data[i]);

            if (slave_data_out == test_data[i])
                $display("Test %0d: Peripheral OK! Received=%h, Expected=%h", i, slave_data_out, test_data[i]);
            else
                $display("Test %0d: Peripheral ERRO! Received=%h, Expected=%h", i, slave_data_out, test_data[i]);
        end

        #100;
        $finish;
    end

    wire sck, cs, mosi, miso, busy, data_out_valid, slave_data_out_valid;
    wire [7:0] data_out, slave_data_out;
    reg  [7:0] data_in, slave_data_in;
    reg        data_in_valid, slave_data_in_valid;

    SPI_Controller #(
        .SPI_BITS_PER_WORD    (8),
        .SPI_MODE             (1),
        .SPI_CLK_FREQ         (1_000_000),
        .SYS_CLK_FREQ         (25_000_000)
    ) u_SPI_Controller (
        .clk                  (clk),                           // 1 bit
        .rst_n                (rst_n),                         // 1 bit
        .sck                  (sck),                           // 1 bit
        .mosi                 (mosi),                          // 1 bit
        .miso                 (miso),                          // 1 bit
        .cs                   (cs),                            // 1 bit
        .data_in_valid        (data_in_valid),                 // 1 bit
        .data_out_valid       (data_out_valid),                // 1 bit
        .data_in              (data_in),                       // ? bits
        .data_out             (data_out),                      // ? bits
        .busy                 (busy)                           // 1 bit
    );

    // SPI Slave
    SPI_Peripheral #(
        .SPI_BITS_PER_WORD (8),
        .SPI_MODE          (1)
    ) u_SPI_Peripheral (
        .clk               (clk),
        .rst_n             (rst_n),
        .sck               (sck),
        .cs                (cs),
        .mosi              (mosi),
        .miso              (miso),
        .data_in_valid     (slave_data_in_valid),
        .data_out_valid    (slave_data_out_valid),
        .busy              (slave_busy),
        .data_in           (slave_data_in),
        .data_out          (slave_data_out)
    );

endmodule
