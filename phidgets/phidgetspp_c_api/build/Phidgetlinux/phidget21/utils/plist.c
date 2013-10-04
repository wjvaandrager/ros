#include "../stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "plist.h"

struct plist_node {
	void *pn_key;
	void *pn_value;
	struct plist_node *pn_next;
	struct plist_node *pn_prev;
};

int
plist_add(void *k, void *v, plist_node_t **root)
{
	plist_node_t *n;

	if (!(n = malloc(sizeof (*n))))
		return 0;
	n->pn_key = k;
	n->pn_value = v;
	if (!*root) {
		n->pn_next = n;
		n->pn_prev = n;
		*root = n;
	} else {
		n->pn_prev = (*root)->pn_prev;
		n->pn_next = (*root);
		(*root)->pn_prev->pn_next = n;
		(*root)->pn_prev = n;
	}
	return 1;
}

void
plist_clear(plist_node_t **root)
{
	plist_node_t *cur;
	plist_node_t *fr;

	cur = *root;
	while (cur) {
		fr = cur;
		cur = cur->pn_next;
		free(fr); fr = NULL;
		if (cur == *root) {
			*root = NULL;
			return;
		}
	}
}

int
plist_remove(void *k, plist_node_t **rootp, void **ov)
{
	plist_node_t *cur;

	cur = *rootp;
	while (cur) {
		if (cur->pn_key == k) {
			if (ov)
				*ov = cur->pn_value;
			cur->pn_prev->pn_next = cur->pn_next;
			cur->pn_next->pn_prev = cur->pn_prev;
			if (cur->pn_next == cur)
				*rootp = NULL;
			else if (*rootp == cur)
				*rootp = cur->pn_next;
			free(cur); cur = NULL;
			return 1;
		}
		cur = cur->pn_next;
		if (cur == *rootp)
			return 0;
	}
	return 0;
}

int
plist_contains(void *k, plist_node_t *root, void **ov)
{
	plist_node_t *cur;

	cur = root;
	while (cur) {
		if (cur->pn_key == k) {
			if (ov)
				*ov = cur->pn_value;
			return 1;
		}
		cur = cur->pn_next;
		if (cur == root)
			return 0;
	}
	return 0;
}

int
plist_walk(plist_node_t *start, int (*func)(const void *k, const void *v,
    void *arg), void *arg)
{
	plist_node_t *cur;
	int res;

	cur = start;
	while (cur) {
		if (!(res = func(cur->pn_key, cur->pn_value, arg)))
			return res;
		cur = cur->pn_next;
		if (cur == start)
			return 1;
	}
	return 1;
}
