# gear_rust_rtic_samples
本リポジトリは、Rustに関する技術書の同人誌[Rustで始める組み込み開発入門 -RP2040 × RTIC × EtherNet/IP-](https://techbookfest.org/product/6h0ub6VdcrPV1Gkcfreke4)のサンプルプロジェクトです。

## hello_rpico
基本となるRTICのプロジェクトです。
ソフトウェアタスクを2つ起動し、ログを出力します。

## timer_irq
タイマ割り込みでログを出力します。

## uart
UARTで通信します。

## spi_master
SPIマスタとして通信します。

## dual_core
コア0とコア1間で通信します。

## pio_pwm
PIOでPWMを制御します。

## ethernetip_driver
コア1でW5500を介して、EtherNet/IPを制御し、共有メモリを読み書きします。
コア0はコア1との共有メモリを読み書きします。
W5500の制御はDMAを介したPIO-SPIで制御します。
