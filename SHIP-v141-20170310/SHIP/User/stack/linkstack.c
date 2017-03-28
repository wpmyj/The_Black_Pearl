#include <stdio.h>
#include <stdlib.h>
#include "linkstack.h"
#include "bsp_usart.h"


linkstack_t *creat_linkstack(void)
{
	linkstack_t *head;
	head = malloc(sizeof(linkstack_t));
	head->next = NULL;
	return head;
}

int push_linkstack(linkstack_t *lsp, data *data_x, data *data_y) //ruzhan 
{
	linkstack_t *newnode;
	
if(((*data_x) <=0.001) ||((*data_y) <=0.001))
	return 0;
	newnode = malloc(sizeof(linkstack_t));
	newnode->x = *data_x;
	newnode->y = *data_y;
	newnode->next = lsp->next;

	lsp->next = newnode;
	
//	printf("\r\npush_linkstack data_x  = %lf   data_y  = %lf --------\r\n", *data_x, *data_y);
	
//	print_linkstack(lsp);
	return 0;
}

int pop_linkstack(linkstack_t *lsp,data *data_x,data *data_y) // chuzhan 
{
	
	linkstack_t *temp;
	
	
//printf("pop_linkstack ------------------1------------\r\n");
	
	if(NULL == lsp->next)
		return -1;

	temp = lsp->next;

	lsp->next = temp->next;

	if((NULL != temp->x )&&(NULL != temp->y))
	{
		*data_x = lsp->next->x;
		*data_y = lsp->next->y;
	}
	else
		return  -1;
	


	//	printf("\r\n pop_linkstack data_x  = %lf   data_y  = %lf --------\r\n", *data_x, *data_y);
		print_linkstack(lsp);
	
			if((*data_x < 0.00001)||(*data_y  < 0.00001))
			{
					free(temp);	
					return -1;
			}
	
	return 0;
}

int get_top_linkstack(linkstack_t *lsp,data *data_x,data *data_y)
{
	if(NULL == lsp->next)
		return -1;
	
	*data_x = lsp->next->x;
	*data_y = lsp->next->y;
	return 0;
}

int empty_linksatck(linkstack_t *lsp)
{
	lsp->next = NULL;
	return 0;
}

int clean_linkstack(linkstack_t *lsp)
{
	linkstack_t *temp1;
	linkstack_t *temp2;

	temp1 = lsp;
	temp2 = temp1->next;

	while(temp2 != NULL){
		temp1 = temp2;
		temp2 = temp2->next;
		free(temp1);
	}

	lsp->next = NULL;

	return 0;
}

int dis_linksatck(linkstack_t **lspp)
{
	clean_linkstack(*lspp);
	free(*lspp);
	*lspp = NULL;
	
	return 0;
}

void print_linkstack(linkstack_t *lsp)
{
	while(NULL != lsp->next){
		lsp = lsp->next;
//		printf("\r\nlsp->xxxxxxx =  %lf ",lsp->x);
//		printf("lsp->yyyyyyy =  %lf\r\n",lsp->y);
	}

//	printf("\r\n-------------end-------\r\n\n");
}



