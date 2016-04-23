#ifndef __MUTIWAY_TREE_H
#define __MUTIWAY_TREE_H
#include <vector>
#include <set>
#include <string>
#include <iostream>
using namespace std;

struct TreeNode 
{
	int 					  vertex;
	int                       weight;
	std::vector<TreeNode*>    child;    
};

TreeNode* create_tree_node(string value);
void connect_tree_nodes(TreeNode* pParent, TreeNode* pChild);
void print_tree_node(TreeNode* pNode);
void print_tree(TreeNode* pRoot);
void destroy_tree(TreeNode* pRoot);
TreeNode* find_node(TreeNode* pRoot, int vertex);

#endif
