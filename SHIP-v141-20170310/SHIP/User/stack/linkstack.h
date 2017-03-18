#ifndef __LINKSTACK_H
#define	__LINKSTACK_H


typedef double data;

typedef struct linkstack{
	data x;
	data y;
	struct linkstack *next;
}linkstack_t;

linkstack_t *creat_linkstack(void);


int push_linkstack(linkstack_t *lsp, data *data_x, data *data_y); //ruzhan 

int pop_linkstack(linkstack_t *lsp,data *data_x,data *data_y); // chuzhan 



int get_top_linkstack(linkstack_t *lsp,data *data_x,data *data_y);

int empty_linksatck(linkstack_t *lsp);


int clean_linkstack(linkstack_t *lsp);

int dis_linksatck(linkstack_t **lspp);


void print_linkstack(linkstack_t *lsp);



#endif /* __ADC_H */
