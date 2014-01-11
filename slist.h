/*
 * slist.h
 *
 *  Created on: 10.07.2013
 *      Author: googl1
 */

#ifndef SLIST_H_
#define SLIST_H_

#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx.h"

typedef struct linked_list_s
{
    uint16_t              data;
    struct linked_list_s *link;
}linked_list_t;

//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

ErrorStatus addItem(linked_list_t **aLinkedList, uint16_t aData);
uint16_t removeLast(linked_list_t **aLinkedList);
uint16_t removeFirst(linked_list_t **aLinkedList);
void clear(linked_list_t **aLinkedList);
uint16_t *list2array(linked_list_t *list, uint16_t len);

#endif /* SLIST_H_ */
