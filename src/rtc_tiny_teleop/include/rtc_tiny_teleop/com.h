#ifndef COM_H_
#define COM_H_


#include <unistd.h>  /* Объявления стандартных функций UNIX write, read, close, usleep*/
#include <fcntl.h>   /* Объявления управления файлами */
#include <termios.h> /* Объявления управления POSIX-терминалом */
#include <cstdio> //for using getchar()
#include <cstdlib> //for using system()
#include <iostream>

using std::cin;
using std::cout;
using std::endl;

#define STEP 20;

/*
 * 'open_port()' - Открывает последовательный порт 1.
 *
 * Возвращает файловый дескриптор при успехе или -1 при ошибке.
 */

class RTK_Tiny
{
	struct termios old_com_opt;
	int fd;

	int open_port();
	int close_port();

	public:
	
	RTK_Tiny()
	{
		fd = open_port();
	}

	~RTK_Tiny()
	{
		close_port();
	}		

	void move(short val1, short val2);
	void echo(unsigned char num);
	short get_value(unsigned char num);
};

int RTK_Tiny::open_port()
{
	const char *port = "/dev/ttyUSB0";
	this->fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (this->fd == -1)
	{
		/*
		* Could not open the port.
		*/
		cout<<"\tOpen "<<port<<": error.\n\n";
		return this->fd;
	} else
	{
    		fcntl(this->fd, F_SETFL, 0);
		cout<<"\tOpen "<<port<<": successful.\n\n";
	}

	/*
	 * Получение текущих опций для порта...
	 */
	tcgetattr(this->fd, &(this->old_com_opt));

	struct termios new_opt = this->old_com_opt;
	/*
	 * Установка скорости передачи в 19200...
	 */
	cfsetispeed(&new_opt, B9600);
	cfsetospeed(&new_opt, B9600);
	/*
	 * Остальные установки...
	 */		
	new_opt.c_cflag |= (CLOCAL | CREAD);	/*Это обеспечит то, что ваша программа не станет
						'владельцем' порта, что приводит к случайному получению
						сигналов управления заданиями или сигналов зависания, а
						также, что драйвер последовательного интерфейса будет
						читать приходящие байты данных (разрешение приема)*/
	new_opt.c_cflag &= ~PARENB;		//Маскирование контроля четности
	new_opt.c_cflag &= ~CSTOPB; 		//Маскирование посылки двух стоповых бита(т.е. один)
	new_opt.c_cflag &= ~CSIZE; 		//Маскирование битов размера символов
	new_opt.c_cflag |= CS8; 		//Установка 8 битов данных
	new_opt.c_cflag &= ~CRTSCTS; 		//Маскирование аппаратного управления потоком
						//передаваемых данных
	new_opt.c_oflag &= ~OPOST;  		//Маскирование обработанныого вывода данных (т.е
						//необработанный)
	new_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	//Выбор неканонического (Raw) ввода
	new_opt.c_iflag &= ~(IXON | IXOFF | IXANY); 		//отмена программного управления потоком
								//передачи данных
	new_opt.c_cc[VMIN]  = 8;		//определяет минимальное число символов для чтения
	new_opt.c_cc[VTIME] = 10;		//определяет время ожидания для чтения первого символа

	/* 
          now clean the modem line
        */
         tcflush(this->fd, TCIFLUSH);

	/*
	 * Установка новых опций для порта...
 	*/	
	tcsetattr(this->fd, TCSANOW, &new_opt);

  return this->fd;
}

int RTK_Tiny::close_port()
{
	tcsetattr(this->fd, TCSANOW, &(this->old_com_opt));
	return close(this->fd);
}

void RTK_Tiny::move(short val1, short val2)
{		
	for(int i = 0; i < 2; ++i)
	{
		unsigned char pak[8]={0x00, 0x4B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00};	
		if(i == 0) 
		{
			cout<<val1<<std::hex<<'\t';
			pak[0] = 0x05;
			pak[5] = static_cast<char>(val1);
			val1 >>= 8;
			pak[4] = static_cast<char>(val1);
		}
		if(i == 1)
		{
			cout<<val2<<std::hex<<'\t';
			pak[0] = 0x06;
			pak[5] = static_cast<char>(val2);
			val2 >>= 8;
			pak[4] = static_cast<char>(val2);
		}
		unsigned char sum = 0;
		for (int i = 0; i < 7; ++i) sum += pak[i];
		pak[7] = 0x00 - sum;
		cout<<"|8\t|";
		for(int i = 0; i < 8; ++i) cout<<static_cast<unsigned int>(pak[i])<<' ';
		cout<<"\t|";

		if (write(this->fd, pak, sizeof(pak))<0) cout<<"Write error\n";	

		unsigned char pak1[8]={0, 0, 0, 0, 0, 0, 0, 0};	
		int size = read(this->fd, pak1, sizeof(pak1));
		if (size<0) cout<<"Read error\n";
		cout<<size<<"\t|";
		for(int i = 0; i < size; ++i) cout<<static_cast<unsigned int>(pak1[i])<<' ';
		usleep(5000);
		cout<<std::dec<<endl;
	}			
}

short RTK_Tiny::get_value(unsigned char num = 0)
{
	cout<<0<<'\t'<<std::hex;
	short result = 0;
	unsigned char pak[8]={(0x05+num), 0x4B, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00};
	//unsigned char pak[8]={(0x05+num), 0x3C, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
	//unsigned char pak[8]={(0x05+num), 0x78, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00};
	//unsigned char pak[8]={0x05, 0x87, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00};	
	unsigned char sum = 0;
	for (int i = 0; i < 7; ++i) sum += pak[i];
	pak[7] = 0x00 - sum;
	cout<<"|8\t|";
	for(int i = 0; i < 8; ++i) cout<<static_cast<unsigned int>(pak[i])<<' ';
	cout<<"\t|";

	if (write(this->fd, pak, sizeof(pak))<0) cout<<"Write error\n";	

	unsigned char pak1[8]={0, 0, 0, 0, 0, 0, 0, 0};
	int size = read(this->fd, pak1, sizeof(pak1));
	if (size<0) cout<<"Read error\n";
	cout<<size<<"\t|";
	for(int i = 0; i < size; ++i) cout<<static_cast<unsigned int>(pak1[i])<<' ';	
	cout<<endl<<std::dec;
	
	if (size == 8)
	{
		unsigned char sum_r = 0, check = 0;
		for (int i = 0; i < 7; ++i) sum_r += pak1[i];
		check = 0x00 - sum_r;	
		if(check != pak1[7]) cout<<"Invalid cheksum\n";
		else 
		{
			int sign = -1;
			if (pak1[6]&0x20) sign = 1;
			//cout<<std::hex<<"pak1[4] = "<<pak1[4]<<endl;
			result = pak1[4];
			//cout<<"result = pak1[4] = "<<result<<endl;
			result<<= 8;
			//cout<<"result<<= 8 = "<<result<<endl;
			//result &= 0xFF00;
			//cout<<"result &= 0xFF00 = "<<result<<endl;
			short cur = pak1[5];
			//cout<<"cur = pak1[5] = "<<cur<<endl;
			//cur &= 0x00FF;
			//cout<<"cur &= 0x00FF = "<<cur<<endl;
			result = (result | cur)*sign;			
			//cout<<"result |= cur = "<<result<<endl;
		}
	}else cout<<"Wrong size\n";
	usleep(5000);
	return result;
}

void RTK_Tiny::echo(unsigned char num = 0)
{
	cout<<0<<'\t'<<std::hex;
	short result = 0;
	unsigned char pak[8]={(0x05+num), 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//ECHO	
	unsigned char sum = 0;
	for (int i = 0; i < 7; ++i) sum += pak[i];
	pak[7] = 0x00 - sum;
	cout<<"|8\t|";
	for(int i = 0; i < 8; ++i) cout<<static_cast<unsigned int>(pak[i])<<' ';
	cout<<"\t|";

	if (write(this->fd, pak, sizeof(pak))<0) cout<<"Write error\n";	

	unsigned char pak1[8]={0, 0, 0, 0, 0, 0, 0, 0};
	int size = read(this->fd, pak1, sizeof(pak1));
	if (size<0) cout<<"Read error\n";
	cout<<size<<"\t|";
	for(int i = 0; i < size; ++i) cout<<static_cast<unsigned int>(pak1[i])<<' ';	
	cout<<endl<<std::dec;	
	usleep(5000);
}

struct termios set_keypress()
{    
	struct termios options; 
	tcgetattr(0, &options);
     
	struct termios new_settings = options;
     
	/* Disable canonical mode, and set buffer size to 1 byte */
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	new_settings.c_cc[VMIN] = 1;
     
	tcsetattr(0,TCSANOW,&new_settings);
	return options;
}
     
void reset_keypress(const struct termios *options)
{
	tcsetattr(0, TCSANOW, options);
}

#endif