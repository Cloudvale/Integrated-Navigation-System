#include "Qeuen.h"

sp_queue queue_init()
{
    sp_queue q;
		memset(q.sp_queue_array, 0, sizeof(q.sp_queue_array));
    q.front = q.rear = 0;
    return q;
}



int queue_empty(sp_queue q)
{
    return q.front == q.rear;
}


int queue_en(sp_queue *q, datatype *e, int buffersize)
{
    /* 队满 */
    if (q -> rear == MAX_QUEUE_SIZE)
        return false;

    /* 入队 */
		memcpy(q -> sp_queue_array[q -> rear],e,buffersize);
//    q -> sp_queue_array[q -> rear][] = e;
//    printf("q.sp_queue_array[%d]=%d\n", q -> rear, e);
    q -> rear += 1;
    return true;

}


int queue_de(sp_queue *q, datatype *e, int buffersize)
{
    /* 队空 */
    if(queue_empty(*q))
        return false;

    /* 出队 */
    q -> rear -= 1;
		memcpy(e,q -> sp_queue_array[0],buffersize);
		for(int i = 0; i <= q->rear; i++)
		{
			memcpy(q -> sp_queue_array[i],q -> sp_queue_array[i+1],buffersize);
		}
//		q -> rear -= 1;
//    *e = q -> sp_queue_array[q -> rear];
    return true;
}


void queue_clear(sp_queue *q)
{
    q -> front = q -> rear = 0;
}


int get_front(sp_queue q, datatype *e)
{
    /* 队空 */
    if(q.front == q.rear)
        return false;

    /* 获取队头元素 */
		memcpy(e,q.sp_queue_array[q.front],100);
//    *e = q.sp_queue_array[q.front];
    return true;
}


int queue_len(sp_queue q)
{
	int len_s = 0;
	len_s = q.rear - q.front;
    return len_s;
}


//void queue_traverse(sp_queue q, void (*visit)(sp_queue q))
//{
//    visit(q);
//}

//void visit(sp_queue q)
//{
//    /* 队空 */
//    if (q.front == q.rear)
//        printf("队列为空\n");

//    int temp = q.front;
//    while(temp != q.rear)
//    {
//        printf("%d ",q.sp_queue_array[temp]);
//        temp += 1;
//    }
//    printf("\n");
//}