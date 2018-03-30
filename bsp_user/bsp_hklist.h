#pragma once

#define HK_LIST_NODE_CNT			(1000U+1)


typedef struct hkListNode {
	uint32_t TimeValue;
	struct hkListNode * pxNext;
	struct hkListNode * pxPrevious;
	void * pvContainer;
}hkListNode_t;

typedef struct __attribute__((packed))
{
	uint32_t TimeValue;
	struct hkListNode * pxNext;
	struct hkListNode * pxPrevious;
}hkMiniListnode_t;

typedef struct __attribute__((packed))
{
	uint32_t uxNumberOfItems;
	struct hkListNode * pxIndex;
	hkMiniListnode_t xListEnd;
} hkList_t;

extern hkList_t  hk_list;



void hk_list_init(hkList_t * pxList);
uint32_t hk_list_remove(hkListNode_t * pxNodeToRemove);
uint32_t hk_list_insert(hkList_t * pxList, uint32_t xValueOfInsertion);
uint32_t hk_list_recover(void);
void * hk_list_find(uint32_t time);