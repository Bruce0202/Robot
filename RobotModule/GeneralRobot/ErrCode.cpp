#include "ErrCode.h"


CErrorQueue::CErrorQueue() 
{
	Set_NULL();
}

CErrorQueue::~CErrorQueue() {}

void CErrorQueue::Set_NULL()//�޸�
{
	m_front = 0;
	m_rear = 0;
	memset(m_ErrQue, 0,ERR_QUE_SIZE * sizeof(ERR_ID));
}

bool CErrorQueue::Is_NULL()
{
	if (m_front == m_rear)
	{
		return true;			 
	}
	return false;
}

bool CErrorQueue::Enter_Queue(ERR_ID newErr) //�޸�
{
	LONG quelen = (m_rear >= m_front ? (m_rear - m_front) : (ERR_QUE_SIZE + m_rear - m_front));
	if (quelen >= ERR_QUE_SIZE - 1)
	{
		return false;
	}
	if (m_rear >= ERR_QUE_SIZE) m_rear = 0;

	m_ErrQue[m_rear] = newErr;
	m_rear++;
	return true;

}

//�Ӷ���ͷȡ���ݣ����ȡlen��������ȡ������
ULONG CErrorQueue::Out_Queue(ERR_ID ErrQue[], ULONG len) //�޸�
{
	ULONG cnt = 0;
	ULONG index = 0;
	for (ULONG i = m_front; i < m_front+len; i++)
	{
		index = (i < ERR_QUE_SIZE) ? i : (i - ERR_QUE_SIZE);
		if (index != m_rear)
		{
			ErrQue[cnt] = m_ErrQue[index];
			m_ErrQue[index] = eNoErr;
			cnt++;
		}
		else
			return cnt;
	}
	return cnt;
}

ULONG CErrorQueue::Clean_Queue(ULONG len)
{
	ULONG cnt = 0;
	for (ULONG i = 0; i < len; i++)
	{
		if (Is_NULL() == false)
		{
			m_ErrQue[m_front] = eNoErr;		//�޸�
			if (++m_front >= ERR_QUE_SIZE)	//�޸�
			{
				m_front = 0;
			}
			cnt++;
		}
		else
			return cnt;
	}
	return cnt;
}