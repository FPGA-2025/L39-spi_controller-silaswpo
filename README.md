# Controlador SPI Master (Multimodo)

O barramento SPI (Serial Peripheral Interface) é um dos protocolos seriais mais utilizados para comunicação entre dispositivos digitais, como sensores, microcontroladores, memórias e displays. Ele é simples, rápido e eficiente, operando com até quatro fios.

## Relembrando

O SPI opera com um Master e um ou mais Slaves. O Master é responsável por gerar o sinal de clock (`SCK`) e controlar o fluxo da comunicação por meio do sinal de seleção de chip (`CS`). Os dados são transmitidos de forma síncrona, com envio (`MOSI`) e recepção (`MISO`) ocorrendo simultaneamente.

## Modos de Operação SPI

O SPI possui quatro modos de operação, definidos pelas combinações dos parâmetros **CPOL** (polarity) e **CPHA** (phase). Esses parâmetros determinam **em qual borda do clock os dados são escritos e amostrados**.

### Tabela de Modos SPI para o Master

| Modo | CPOL | CPHA | Borda de leitura (MISO) | Borda de escrita (MOSI) |
| ---- | ---- | ---- | ----------------------- | ----------------------- |
| 0    | 0    | 0    | Subida                  | Descida                 |
| 1    | 0    | 1    | Descida                 | Subida                  |
| 2    | 1    | 0    | Descida                 | Subida                  |
| 3    | 1    | 1    | Subida                  | Descida                 |


## Objetivo

Neste laboratório, você irá implementar um **controlador SPI Master** em Verilog, configurável para operar nos 4 modos descritos acima. O controlador deve permitir envio e recepção de dados em modo **full-duplex**, com sinal de clock configurável e controle automático de `CS`.


## Módulo SPI Controller

```verilog
module SPI_Controller #(
    parameter SPI_BITS_PER_WORD = 8,
    parameter SPI_MODE          = 0,          // 0: CPOL=0, CPHA=0; 1: CPOL=0, CPHA=1; 2: CPOL=1, CPHA=0; 3: CPOL=1, CPHA=1
    parameter SPI_CLK_FREQ      = 1_000_000,  // 1MHz
    parameter SYS_CLK_FREQ      = 25_000_000  // 25MHz
) (
    input  wire clk,
    input  wire rst,

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
    // Sua implementação aqui
endmodule
```

## Requisitos

1. O controlador deve iniciar a transação quando `data_in_valid` for 1
2. Durante a transação, o sinal `cs` deve ser mantido em nível baixo
3. O sinal `sck` deve obedecer à polaridade (CPOL) e frequência desejada
4. O envio e recepção devem respeitar a fase do clock (CPHA)
5. O sinal `data_out_valid` deve ser ativado por **1 ciclo de clock** ao final da transmissão
6. A saída `busy` deve indicar se o módulo está ocupado realizando uma transação


## Execução da atividade

Siga o modelo de módulo já fornecido e utilize o testbench e scripts de execução para sua verificação. Em seguida, implemente o circuito de acordo com as especificações e, se necessário, crie outros testes para verificá-lo.

Uma vez que estiver satisfeito com o seu código, execute o script de testes com `./run-all.sh`. Ele mostrará na tela `ERRO` em caso de falha ou `OK` em caso de sucesso.


## Entrega

Realize um *commit* no repositório do **GitHub Classroom**. O sistema de correção automática irá validar sua implementação e atribuir uma nota com base nos testes.


## Dicas

* Use uma FSM (máquina de estados) para gerenciar as fases da transação
* Divida o clock do sistema para gerar `sck` com a frequência correta
* Use shift registers para `data_in` e `data_out`
* Detecção de bordas pode ser feita com registradores de sincronização
* Certifique-se de sincronizar sinais assíncronos, como `data_in_valid`

