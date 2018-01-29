#include <cstring>
#include "MyQueue.h"

MyQueue::MyQueue(void)
{
	myQue.front = NULL;
	myQue.rear = NULL;
}

bool MyQueue::QIsEmpty()
{
	if(myQue.front == NULL)		return true;
	else						return false;
}

void MyQueue::Push(Data data)
{
	Node * newNode = new Node;
	newNode->next = NULL;
	newNode->data = new char[strlen(data)+1];
	strcpy(newNode->data, data);

	if(QIsEmpty()){
		myQue.front = newNode;
		myQue.rear = newNode;
	}
	else{
		myQue.rear->next = newNode;
		myQue.rear = newNode;
	}
}

void MyQueue::Pop(Data data)
{
	Node * delNode;

	if(QIsEmpty()){
		return;
	}

	delNode = myQue.front;
	strcpy(data, delNode->data);
	myQue.front = myQue.front->next;

	delete [](delNode->data);
	delete delNode;
}
