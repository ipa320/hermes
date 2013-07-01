class GraspHand
{
public:
	int init(int port);
	int open_port(int port);
	int configure_port(int fd);
	int sendByte(int fd, char data);
	void strongGrip(int fd);
	void doPeineta(int fd);
	void executeGrasp(int type, int force);

protected:
	int file_description;	// file description for the serial port


};
