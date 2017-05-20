#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QList>
#include <QString>
#include <QMainWindow>
#include <QKeyEvent>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QCamera>
#include <QCameraInfo>
#include <QCameraViewfinder>

#define BUFFER_DATA_WIDTH     16

#define RC_COMMAND_IMU_CALI   0x01
#define RC_COMMAND_ARM_LOCK   0x02
#define RC_COMMAND_CONTROL    0x03
#define RC_COMMAND_INVALID    0x04

#define RC_DATA_LENGTH_VOID   0x00
#define RC_DATA_LENGTH_BYTE_8 0x08

#define RC_CAMERA_NAME        "@device:pnp:\\\\?\\usb#vid_18ec&pid_5555&mi_00#8&19de01dc&0&0000#{65e8773d-8f56-11d0-a3b9-00a0c9223196}\\global"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public:
    void closeSerialPort(void);
    void setSerialBaudRate(uint32_t baud_rate);
    void setSerialPortName(QString &port_name);
    uint8_t writeDataToSerial(const char *buffer, uint8_t size);
    bool clearSerialPort(void);
    bool openSerialPort(void);
    QSerialPort::BaudRate getSerialBaudRate(void) const;
    QSerialPort::DataBits getSerialDataBits(void) const;
    QSerialPort::StopBits getSerialStopBits(void) const;
protected:
    void keyPressEvent(QKeyEvent *event);
private slots:
    void on_push_button_clear_clicked(void);
    void on_push_button_close_clicked(void);
    void on_push_button_open_clicked(void);
    void on_push_button_send_clicked(void);
    void readDataFromSerial(void);
private:
    void addBits8ToBuffer(uint8_t data);
    void addBits16ToBuffer(uint16_t data);
    void sendBufferToSerialPort(void);
    uint16_t getConstrainedValue(uint16_t data);
private:
    char                   send_buffer_[BUFFER_DATA_WIDTH];
    uint8_t                send_checksum_;
    uint8_t                send_count_;
    uint8_t                rc_command_;
    uint8_t                data_step_;
    uint16_t               data_roll_;
    uint16_t               data_pitch_;
    uint16_t               data_yaw_;
    uint16_t               data_thrust_;
    uint32_t               serial_baud_rate_;
    QString                serial_port_name_;
    QSerialPort           *serial_port_;
    QList<QSerialPortInfo> serial_ports_info_;
    QCamera               *camera_;
    QList<QCameraInfo>     cameras_info_;
    QCameraViewfinder     *camera_view_finder_;
    Ui::MainWindow        *ui;
};

#endif // MAINWINDOW_H
