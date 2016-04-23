#include "multiway_tree.h"

TreeNode* creat_tree_node(int vertex)
{
	TreeNode* pNode = new TreeNode();
	pNode->vertex 	= vertex;
	pNode->weight   = 0;
	return pNode;
}

TreeNode* find_node(TreeNode* pRoot, int vertex)
{
    TreeNode *node = NULL;
    if(pRoot->vertex == vertex) return pRoot;    
    vector<TreeNode*>::iterator iter = pRoot->child.begin();
    while(NULL == node && iter < pRoot->child.end())
    {
        node = find_node(*iter, vertex);
        ++iter;
    }
    return node;
}

void connect_tree_nodes(TreeNode* pParent, TreeNode* pChild)
{
	if(pParent != NULL)
	{
		pParent->child.push_back(pChild);
	}
}

void print_tree_node(TreeNode* pNode)
{
	if(pNode != NULL)
	{
		//printf("its children is as the following:\n");
		std::vector<TreeNode*>::iterator iter = pNode->child.begin();
		while(iter < pNode->child.end())
		{
			if(*iter != NULL)
				cout << (*iter)->vertex << endl;
			++iter;
		}
		//printf("\n");
	}
	else
	{
		//printf("this node is null.\n");
	}
	//printf("\n");
}

void print_tree(TreeNode* pRoot)
{
	print_tree_node(pRoot);
	if(pRoot != NULL)
	{
		std::vector<TreeNode*>::iterator iter = pRoot->child.begin();
		while(iter < pRoot->child.end())
		{
			print_tree(*iter);
			++iter;
		}
	}
}

void destroy_tree(TreeNode* pRoot)
{
	if(pRoot != NULL)
	{
		std::vector<TreeNode*>::iterator iter = pRoot->child.begin();
		while(iter < pRoot->child.end())
		{
			destroy_tree(*iter);
			++iter;
		}
		delete pRoot;
	}
}
