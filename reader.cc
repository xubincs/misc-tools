#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <poll.h>

#include <iostream>

using namespace std;

int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        cerr << "error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
//    tty.c_cc[VMIN]  = 0;            // read doesn't block
//    tty.c_cc[VTIME] = 100;            // seconds times by 0.1

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        cerr << "error " << errno << " from tcsetattr: " << strerror(errno) << endl;
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        cerr << "Please specify port name" << endl;
        return 1;
    }

    const char *portname = argv[1];

//    int fd = open(portname, O_RDONLY | O_NOCTTY | O_SYNC);
    int fd = open(portname, O_RDONLY | O_NDELAY);
    if (fd < 0) {
        cout << "error " << errno << " opening " << portname << " : " << strerror (errno) << endl;
        return 2;
    }

    tcflush(fd, TCIFLUSH);
    set_interface_attribs (fd, B115200, 0);

    struct pollfd fds;
    fds.fd = fd;
    fds.events = POLLRDNORM;

    char buf[80];
    while (true) {

//        int ret = poll(&fds, 1, 60000);
//        if (ret == -1)
//            cout << "error" << endl;
//        else if (ret == 0) {
//            cout << "Time is up" << endl;
//            goto out;
//        } else {
            memset(buf, 0, sizeof buf);
            read(fd, buf, sizeof buf);
            cout << buf;
            cout.flush();
//        }
    }

out:
    close(fd);
    return 0;
}
