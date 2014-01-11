/*
 * slist.c
 *
 *  Created on: 10.07.2013
 *      Author: googl1
 */

#include "slist.h"

ErrorStatus addItem(linked_list_t **aLinkedList, uint16_t aData)
{
    linked_list_t *temp = *aLinkedList;

    if(*aLinkedList==NULL)
    {
        *aLinkedList=malloc(sizeof(linked_list_t));
        if (*aLinkedList == NULL)
        	return ERROR;
        temp = *aLinkedList;
    }
    else
    {
        while((temp->link)!=NULL)
        {
            temp=temp->link;
        }
        temp->link = malloc(sizeof(linked_list_t));
        if (temp->link == NULL)
                	return ERROR;
        temp=temp->link;
    }
    temp->data = aData;
    temp->link  = NULL;

    return !ERROR;
}

//--------------------------------------------------------------
// copy all values from an slist to an array
//--------------------------------------------------------------
uint16_t *list2array(linked_list_t *list, uint16_t len)
{
	uint16_t *array;
	linked_list_t *node = list;
	uint32_t i,b;


	array = malloc(len * sizeof(uint16_t));
    if(NULL == array) {
       return NULL;
    }
	node = list;

	for (i = 0; i < len; i++) {
		b = node->data;
		array[i] = b;
		node = node->link;
	}

	return array;
}

uint16_t removeFirst(linked_list_t **list)
{
	uint16_t ret = (*list)->data;
	linked_list_t* first = *list;

	if (first == NULL)
		return 0;

	*list = (*list)->link;
	free(first);
	first = NULL;
	return ret;
}

uint16_t removeLast(linked_list_t **aLinkedList)
{
	linked_list_t *temp;
	uint16_t ret;

	temp = *aLinkedList;
	if(temp == NULL)
		return 0;

	if (temp->link == NULL) {
		ret = temp->data;
		free(*aLinkedList);
		*aLinkedList = NULL;
		return ret;
	}

	while((temp->link->link)!=NULL)
	{
		temp=temp->link;
	}
	ret = temp->link->data;
	free(temp->link);
	temp->link = NULL;
	return ret;
}

void clear(linked_list_t **aLinkedList)
{
	while(*aLinkedList != NULL)
		removeLast(aLinkedList);
}
