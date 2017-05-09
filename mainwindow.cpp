#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->line_edit_roll->setText("1500");
    ui->line_edit_pitch->setText("1500");
    ui->line_edit_yaw->setText("1500");
    ui->line_edit_thrust->setText("1500");

    ui->statusBar->showMessage("Open software successfully!", 3000);

    this->setWindowTitle("Breeze Remote Control");
    this->setFixedSize(430, 260);

    rc_command_ = RC_COMMAND_INVALID;

    data_roll_   = 1500;
    data_pitch_  = 1500;
    data_yaw_    = 1500;
    data_thrust_ = 1500;
    data_step_   = 10;

    send_count_ = 0;

    for (uint8_t i = 0; i < BUFFER_DATA_WIDTH; i++) {
        send_buffer_[i] = 0;
    }

    serial_baud_rate_ = 115200;
    serial_port_name_ = "COM3";
    serial_port_      = new QSerialPort();
}

MainWindow::~MainWindow()
{
    delete ui;
    this->closeSerialPort();
    delete serial_port_;
}

void MainWindow::closeSerialPort(void)
{
    if (serial_port_->isOpen()) {
        serial_port_->close();
    }
}

void MainWindow::setSerialBaudRate(uint32_t baud_rate)
{
    serial_baud_rate_ = baud_rate;
}

void MainWindow::setSerialPortName(QString &port_name)
{
    serial_port_name_ = port_name;
}

uint8_t MainWindow::readDataFromSerial(char *buffer, uint8_t size,
                                       uint16_t timeout)
{
    uint8_t length = 0;

    forever {
        int number = serial_port_->read(&buffer[length], size - length);
        if (number == -1) {
            return -1;
        }
        else if (number == 0 && !serial_port_->waitForReadyRead(timeout)) {
            return -2;
        }
        else {
            length += number;
            if (size == length) {
                break ;
            }
        }
    }

    return length;
}

uint8_t MainWindow::writeDataToSerial(const char *buffer, uint8_t size)
{
    uint8_t length = 0;

    forever {
        int number = serial_port_->write(&buffer[length], size - length);
        if (number == -1) {
            return -1;
        }
        else {
            length += number;
            if (size == length) {
                break ;
            }
        }
    }

    return length;
}

bool MainWindow::clearSerialPort(void)
{
    if (serial_port_->isOpen()) {
        serial_port_->clear();
        this->closeSerialPort();
        return this->openSerialPort();
    }
    else {
        return false;
    }
}

bool MainWindow::openSerialPort(void)
{
    if (serial_port_->isOpen()) {
        return true;
    }

    serial_port_->setPortName(serial_port_name_);
    serial_port_->setBaudRate(this->getSerialBaudRate());
    serial_port_->setParity(QSerialPort::NoParity);
    serial_port_->setDataBits(QSerialPort::Data8);
    serial_port_->setStopBits(QSerialPort::OneStop);
    serial_port_->setFlowControl(QSerialPort::NoFlowControl);
    serial_port_->setReadBufferSize(1024);

    return serial_port_->open(QSerialPort::ReadWrite);
}

QSerialPort::BaudRate MainWindow::getSerialBaudRate(void)
{
    switch (serial_baud_rate_) {
        case 1200: {
            return QSerialPort::Baud1200;
        }
        case 2400: {
            return QSerialPort::Baud2400;
        }
        case 4800: {
            return QSerialPort::Baud4800;
        }
        case 9600: {
            return QSerialPort::Baud9600;
        }
        case 19200: {
            return QSerialPort::Baud19200;
        }
        case 38400: {
            return QSerialPort::Baud38400;
        }
        case 57600: {
            return QSerialPort::Baud57600;
        }
        case 115200: {
            return QSerialPort::Baud115200;
        }
        default: {
            return QSerialPort::UnknownBaud;
        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
        case Qt::Key_U: {
            rc_command_ = RC_COMMAND_IMU_CALI;
            break ;
        }
        case Qt::Key_O: {
            rc_command_ = RC_COMMAND_ARM_LOCK;
            break ;
        }
        case Qt::Key_W: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_thrust_ += data_step_;
            break ;
        }
        case Qt::Key_S: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_thrust_ -= data_step_;
            break ;
        }
        case Qt::Key_A: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_yaw_ += data_step_;
            break ;
        }
        case Qt::Key_D: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_yaw_ -= data_step_;
            break ;
        }
        case Qt::Key_I: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_pitch_ += data_step_;
            break ;
        }
        case Qt::Key_K: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_pitch_ -= data_step_;
            break;
        }
        case Qt::Key_J: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_roll_ += data_step_;
            break ;
        }
        case Qt::Key_L: {
            rc_command_ = RC_COMMAND_CONTROL;
            data_roll_ -= data_step_;
            break ;
        }
        case Qt::Key_Q: {
            data_step_ += 1;
            if (data_step_ >= 20) {
                data_step_ = 20;
            }
            ui->statusBar->showMessage("Data Step:" +
                                       QString::number(data_step_, 10));
            return ;
        }
        case Qt::Key_E: {
            data_step_ -= 1;
            if (data_step_ <= 1) {
                data_step_ = 1;
            }
            ui->statusBar->showMessage("Data Step:" +
                                       QString::number(data_step_, 10));
            return ;
        }
        case Qt::Key_X: {
            this->sendBufferToSerialPort();
            return ;
        }
        case Qt::Key_M: {
            rc_command_ = RC_COMMAND_IMU_CALI;
            ui->statusBar->showMessage("RC Command:" +
                                       QString::number(rc_command_, 10));
            return ;
        }
        case Qt::Key_Comma: {
            rc_command_ = RC_COMMAND_ARM_LOCK;
            ui->statusBar->showMessage("RC Command:" +
                                       QString::number(rc_command_, 10));
            return ;
        }
        case Qt::Key_Period: {
            rc_command_ = RC_COMMAND_CONTROL;
            ui->statusBar->showMessage("RC Command:" +
                                       QString::number(rc_command_, 10));
            return ;
        }
        default: {
            qDebug("Invalid key!");
            return ;
        }
    }

    data_roll_   = this->getConstrainedValue(data_roll_);
    data_pitch_  = this->getConstrainedValue(data_pitch_);
    data_yaw_    = this->getConstrainedValue(data_yaw_);
    data_thrust_ = this->getConstrainedValue(data_thrust_);

    this->sendBufferToSerialPort();

    ui->line_edit_roll->setText(QString::number(data_roll_, 10));
    ui->line_edit_pitch->setText(QString::number(data_pitch_, 10));
    ui->line_edit_yaw->setText(QString::number(data_yaw_, 10));
    ui->line_edit_thrust->setText(QString::number(data_thrust_, 10));

    ui->statusBar->showMessage("Data Step:" +
                               QString::number(data_step_, 10) +
                               " | " + "RC Command:" +
                               QString::number(rc_command_, 10));
}

void MainWindow::on_push_button_close_clicked(void)
{
    const char *string = "Close serial port successfully!";

    this->closeSerialPort();
    ui->statusBar->showMessage(string, 3000);
    qDebug("%s", string);
}

void MainWindow::on_push_button_open_clicked(void)
{
    const char *string_a = "Open serial port successfully!";
    const char *string_b = "Failed to open serial port!";

    if (this->openSerialPort()) {
        ui->statusBar->showMessage(string_a, 3000);
        qDebug("%s", string_a);
    }
    else {
        ui->statusBar->showMessage(string_b, 3000);
        qDebug("%s", string_b);
    }
}

void MainWindow::on_push_button_send_clicked(void)
{
    this->sendBufferToSerialPort();
}

void MainWindow::addBits8ToBuffer(uint8_t data)
{
    send_buffer_[send_count_++] = data;
    send_checksum_ ^= data;
}

void MainWindow::addBits16ToBuffer(uint16_t data)
{
    this->addBits8ToBuffer((uint8_t)(data & 0XFF));
    this->addBits8ToBuffer((uint8_t)(data >> 8));
}

void MainWindow::sendBufferToSerialPort(void)
{
    for (int i = 0; i < BUFFER_DATA_WIDTH; i++) {
        send_buffer_[i] = 0;
    }

    send_count_ = 0;

    this->addBits8ToBuffer(0XAA);
    this->addBits8ToBuffer(0XAA);

    send_checksum_ = 0;

    if (rc_command_ == RC_COMMAND_IMU_CALI) {
        addBits8ToBuffer(RC_COMMAND_IMU_CALI);
        addBits8ToBuffer(RC_DATA_LENGTH_VOID);
    }
    else if (rc_command_ == RC_COMMAND_ARM_LOCK) {
        addBits8ToBuffer(RC_COMMAND_ARM_LOCK);
        addBits8ToBuffer(RC_DATA_LENGTH_VOID);
    }
    else if (rc_command_ == RC_COMMAND_CONTROL) {
            this->addBits8ToBuffer(RC_COMMAND_CONTROL);
            this->addBits8ToBuffer(RC_DATA_LENGTH_BYTE_8);
            this->addBits16ToBuffer(data_roll_);
            this->addBits16ToBuffer(data_pitch_);
            this->addBits16ToBuffer(data_yaw_);
            this->addBits16ToBuffer(data_thrust_);
    }
    else {
        qDebug("Invalid command!");
        return ;
    }

    this->addBits8ToBuffer(send_checksum_);

    if (serial_port_->isOpen()) {
        this->writeDataToSerial(send_buffer_, BUFFER_DATA_WIDTH);
        qDebug("Send Data to serial port!");
    }
    else {
        qDebug("Fail to open serial port!");
    }

    for (int i = 0; i < BUFFER_DATA_WIDTH; i++) {
        qDebug("%d", send_buffer_[i]);
    }
}

uint16_t MainWindow::getConstrainedValue(uint16_t data)
{
    if (data >= 2000) {
        data = 2000;
    }
    else if (data <= 1000) {
        data = 1000;
    }
    else {
        ;
    }

    return data;
}
