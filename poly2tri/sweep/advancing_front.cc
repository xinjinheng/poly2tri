/*
 * Poly2Tri Copyright (c) 2009-2018, Poly2Tri Contributors
 * https://github.com/jhasse/poly2tri
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "advancing_front.h"

#include <cassert>

namespace p2t {

AdvancingFront::AdvancingFront(Node& head, Node& tail)
{
  head_ = &head;
  tail_ = &tail;
  search_node_ = &head;
  bst_root_ = nullptr;
  
  // Initialize BST with initial nodes
  AddNodeToBST(&head);
  AddNodeToBST(&tail);
}

Node* AdvancingFront::LocateNode(double x)
{
  // Use BST to find the closest node
  Node* node = BSTSearch(bst_root_, x);
  if (!node) {
    return nullptr;
  }
  
  // Update search node for next time
  search_node_ = node;
  
  return node;
}

Node* AdvancingFront::FindSearchNode(double x)
{
  return BSTSearch(bst_root_, x);
}

Node* AdvancingFront::LocatePoint(const Point* point)
{
  const double px = point->x;
  Node* node = FindSearchNode(px);
  const double nx = node->point->x;

  if (px == nx) {
    if (point != node->point) {
      // We might have two nodes with same x value for a short time
      if (point == node->prev->point) {
        node = node->prev;
      } else if (point == node->next->point) {
        node = node->next;
      } else {
        assert(0);
      }
    }
  } else if (px < nx) {
    while ((node = node->prev) != nullptr) {
      if (point == node->point) {
        break;
      }
    }
  } else {
    while ((node = node->next) != nullptr) {
      if (point == node->point)
        break;
    }
  }
  if(node) search_node_ = node;
  return node;
}

AdvancingFront::~AdvancingFront()
{
  // Clean up BST
  // TODO: Implement BST cleanup
}

void AdvancingFront::AddNodeToBST(Node* node)
{
  bst_root_ = BSTInsert(bst_root_, node);
}

void AdvancingFront::RemoveNodeFromBST(Node* node)
{
  bst_root_ = BSTDelete(bst_root_, node->value);
}

BSTNode* AdvancingFront::BSTInsert(BSTNode* root, Node* node)
{
  if (root == nullptr) {
    return new BSTNode(node);
  }
  
  if (node->value < root->front_node->value) {
    root->left = BSTInsert(root->left, node);
  } else if (node->value > root->front_node->value) {
    root->right = BSTInsert(root->right, node);
  } else {
    // Handle duplicate values (shouldn't happen in advancing front)
    // We'll just add to the right subtree
    root->right = BSTInsert(root->right, node);
  }
  
  return root;
}

BSTNode* AdvancingFront::BSTDelete(BSTNode* root, double x)
{
  if (root == nullptr) {
    return root;
  }
  
  if (x < root->front_node->value) {
    root->left = BSTDelete(root->left, x);
  } else if (x > root->front_node->value) {
    root->right = BSTDelete(root->right, x);
  } else {
    // Node with only one child or no child
    if (root->left == nullptr) {
      BSTNode* temp = root->right;
      delete root;
      return temp;
    } else if (root->right == nullptr) {
      BSTNode* temp = root->left;
      delete root;
      return temp;
    }
    
    // Node with two children: Get the inorder successor (smallest in the right subtree)
    BSTNode* temp = BSTFindMin(root->right);
    
    // Copy the inorder successor's data to this node
    root->front_node = temp->front_node;
    
    // Delete the inorder successor
    root->right = BSTDelete(root->right, temp->front_node->value);
  }
  
  return root;
}

Node* AdvancingFront::BSTSearch(BSTNode* root, double x)
{
  if (root == nullptr || std::abs(root->front_node->value - x) < EPSILON) {
    return root ? root->front_node : nullptr;
  }
  
  if (x < root->front_node->value) {
    return BSTSearch(root->left, x);
  } else {
    return BSTSearch(root->right, x);
  }
}

BSTNode* AdvancingFront::BSTFindMin(BSTNode* root)
{
  BSTNode* current = root;
  while (current && current->left != nullptr) {
    current = current->left;
  }
  return current;
}

} // namespace p2t
