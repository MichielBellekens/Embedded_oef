
struct node {
struct node * next;
char data[30];
};

struct node node10 =
	{
		NULL,
		0x64, 0x6F, 0x6F, 0x72, 0x6C, 0x6F, 0x70, 0x65, 0x6E, 0x2E, 0x00
};

struct node node9 =
	{
		&node10,
		0x6C, 0x69, 0x6A, 0x73, 0x74, 0x20, 0x00
};

struct node node8 =
	{
		&node9,
		0x76, 0x6F, 0x6C, 0x6C, 0x65, 0x64, 0x69, 0x67, 0x65, 0x20, 0x00
};

struct node node7 =
	{
		&node8,
		0x64, 0x65, 0x20, 0x00
};

struct node node6 =
	{
		&node7,
		0x6A, 0x65, 0x20, 0x00
};

struct node node5 =
	{
		&node6,
		0x68, 0x65, 0x62, 0x20, 0x00
};

struct node node4 =
	{
		&node5,
		0x44, 0x61, 0x6E, 0x20, 0x00
};

struct node node3 =
	{
		&node4,
		0x4C, 0x65, 0x7A, 0x65, 0x6E, 0x2E, 0x20, 0x00
};

struct node node2 =
	{
		&node3,
		0x4B, 0x61, 0x6E, 0x20, 0x00
};

struct node node1 =
	{
		&node2,
		0x44, 0x69, 0x74, 0x20, 0x00
};

struct node node0 =
	{
		&node1,
		0x41, 0x6C, 0x73, 0x20, 0x6A, 0x65, 0x20, 0x00
};

