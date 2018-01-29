#pragma once

typedef char * Data;

class MyQueue
{
public:
	MyQueue(void);
	bool QIsEmpty();
	void Push(Data data);
	void Pop(Data data);

private:
	typedef struct _node{
		Data data;
		struct _node * next;
	} Node;

	typedef struct _lQueue{
		Node * front;
		Node * rear;
	} LQueue;

	LQueue myQue;
};
