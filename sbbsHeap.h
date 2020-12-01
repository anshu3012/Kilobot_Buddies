/*  Michael Otte, University of Colorado, 9-22-08
 *
// The MIT License (MIT)
//
// Copyright Copyright 2008-2019, Michael Otte
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
 *
 *
 *
 *  basic self balancing binary search heap implimentation
 *
 *  NOTE: Modified in 2018 to make into a more general C++ class
 *  NOTE: Modified in 2019 to make compatable with matlab C mex
 *        and for specific use with ssspGraphNode (change from C++ to C)
 *        which makes less compatible in general 
 *        2018 C++ version still exists, just not here.
 *  NOTE: Modified 2020 to rip out mex dependent stuff for use in a more 
 *        general setting.
 */

#ifndef __SBBSHEAP_H__
#define __SBBSHEAP_H__


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// NOTE: use 
// #define nodeT yourNodeType
// to make this compatible with exisitng nodes 
// (since C does not have templates) 

#define getInfinity() INFINITY


struct sbbsHeap 
{ 
  double* heapCost;          // cost values
  nodeT** heapNode;          // pointers to the corresponding map nodes
                             // (or whatever is stored in the heap)

  int parentOfLast;          // stores the index of the parent of the last node
  int tempInd;               // used to help swap nodes
  double tempCost;           // used to help swap nodes
  nodeT*  tempNode;          // used to help swap nodes

  int capacity;              // the number of things this heap can store without
                             // being increased in size

  int indexOfLast;           // the index of the last node in the heap array

};
       
typedef struct sbbsHeap sbbsHeap;


// construct and initializes heap to have safe default values  
// and return a pointer to the heap
// assumes a reserve size of heapCapacity
sbbsHeap* sbbsHeapConstruct(int heapCapacity)
{
  //mexPrintfSafe("building a heap\n");

  sbbsHeap* H = (sbbsHeap*)malloc(sizeof(sbbsHeap));

  H->heapCost = (double*)calloc(heapCapacity, sizeof(double));
  //mexMakeMemoryPersistent(H->heapCost);
  
  H->heapNode = (nodeT**)calloc(heapCapacity, sizeof(nodeT*));
  //mexMakeMemoryPersistent(H->heapNode);
  
  H->indexOfLast = -1;
  H->parentOfLast = -1;
  H->tempCost = getInfinity();
  H->tempNode = NULL;
  H->capacity = heapCapacity;
  int i;
  for(i = 0; i < heapCapacity; i++)
  {
    H->heapCost[i] = getInfinity();
    H->heapNode[i] = NULL;
  }
  
  return H;
}

// destructor, cleans up and deletes the memory used by the heap
void sbbsHeapDestruct(sbbsHeap* H)
{
  //log_message_from_sim("destruct");

  //mexPrintfSafe("deleting a heap\n");
    
  if(H->heapCost != NULL)
  { 
    for(int i = 0; i <= H->indexOfLast; i++)
    {
      H->heapCost[i] = getInfinity();
    }  
    free(H->heapCost);
  }
  H->heapCost = NULL;
  
  if(H->heapNode != NULL)
  {
    for(int i = 0; i <= H->indexOfLast; i++)
    {
      H->heapNode[i]->heapIndex = -1;
      H->heapNode[i]->inHeap = false;
      H->heapNode[i] = NULL;
    }    
    free(H->heapNode);
  }
  H->heapNode = NULL;
  
  H->indexOfLast = -1;
  H->parentOfLast = -1;
  H->tempInd = -1;
  H->tempNode = NULL;
  H->tempCost = getInfinity();
  
  free(H);
}

// resets the heap to be empty
void emptyHeap(sbbsHeap* H)
{
  for(int i = 0; i <= H->indexOfLast; i++)
  {
    H->heapNode[i]->heapIndex = -1;
    H->heapNode[i]->inHeap = false;
    H->heapCost[i] = getInfinity();
  }
    
  H->indexOfLast = -1;
  H->parentOfLast = -1;
  H->tempInd = -1;
  H->tempNode = NULL;
  H->tempCost = getInfinity();
}

// increases the heap size by a factor of two
// this should only be used for debugging as it will really slow down the code
void increaseHeapSize(sbbsHeap* H)
{
  int oldCapacity = H->capacity;
  H->capacity = H->capacity * 2;
  if(H->capacity < 100)
  {
    H->capacity = 100;
  }
  
  double* newHeapCost = (double*)calloc(H->capacity, sizeof(double));
  
  
  nodeT** newHeapNode = (nodeT**)calloc(H->capacity, sizeof(nodeT*)); 
  
  memcpy(newHeapCost, H->heapCost, oldCapacity*sizeof(double));
  memcpy(newHeapNode, H->heapNode, oldCapacity*sizeof(nodeT*));

  free(H->heapCost);
  free(H->heapNode);

  H->heapCost = newHeapCost;
  H->heapNode = newHeapNode;

  //mexPrintf("heap can now hold up to %d items\n", capacity);
}


// compares a node n with its parent, and switches them if the parent's
// cost is more than the node's cost. Repeats if a switch happens.
// returns the index that the node ended up at
int bubbleUp(sbbsHeap* H, int n)
{
  H->tempInd = (n-1)/2;
  while(n != 0 &&  H->heapCost[H->tempInd] > H->heapCost[n])
  {
     // swap costs 
     H->tempCost = H->heapCost[H->tempInd]; 
     H->heapCost[H->tempInd] = H->heapCost[n];
     H->heapCost[n] = H->tempCost;
     
     // swap graph node pointers
     H->tempNode = H->heapNode[H->tempInd];
     H->heapNode[H->tempInd] = H->heapNode[n];
     H->heapNode[n] = H->tempNode;
     
     // update graph node heap index values
     H->heapNode[H->tempInd]->heapIndex = H->tempInd;
     H->heapNode[n]->heapIndex = n;
     
     // get new node and parent indicies
     n = H->tempInd;
     H->tempInd = (n-1)/2;
  } 
  return n;  
}

// compares a node n with its children, and switches them if a child's cost
// is less than the node's cost. Repeats if a switch happens. 
void bubbleDown(sbbsHeap* H, int n)
{
  if(H->indexOfLast < 2*n+1)  
  {
    return;   
  }
  else if(H->indexOfLast == 2*n+1)
  {
    H->tempInd = 2*n+1;
  }
  else if(H->heapCost[2*n+1] < H->heapCost[2*n+2])   // find child with smallest value
  {
    H->tempInd = 2*n+1;
  }
  else
  {
    H->tempInd = 2*n+2; 
  }
  
  while(n <= H->parentOfLast && H->heapCost[H->tempInd] < H->heapCost[n])
  {
     // swap costs 
     H->tempCost = H->heapCost[H->tempInd]; 
     H->heapCost[H->tempInd] = H->heapCost[n];
     H->heapCost[n] = H->tempCost;
     
     // swap graph node pointers
     H->tempNode = H->heapNode[H->tempInd];
     H->heapNode[H->tempInd] = H->heapNode[n];
     H->heapNode[n] = H->tempNode;  
      
     // update graph node heap index values
     H->heapNode[H->tempInd]->heapIndex = H->tempInd;
     H->heapNode[n]->heapIndex = n;
     
     // get new node and child indicies
     n = H->tempInd;
     if(H->indexOfLast < 2*n+1)  
     {
       return;   
     }
     else if(H->indexOfLast == 2*n+1)
     {
       H->tempInd = 2*n+1;
     }
     else if(H->heapCost[2*n+1] < H->heapCost[2*n+2])
     {
       H->tempInd = 2*n+1;
     }
     else
     {
       H->tempInd = 2*n+2;  
     }
  }   
}

// add thisNode to the heap, with the key value
void addToHeap(sbbsHeap* H, nodeT* thisNode, double keyValue)
{
  //log_message_from_sim("add");

  if(H->indexOfLast == H->capacity-1)
  {
    increaseHeapSize(H);
  }

  if(!thisNode->inHeap)
  {
    H->indexOfLast++;
    H->parentOfLast = (H->indexOfLast-1)/2;
    H->heapNode[H->indexOfLast] = thisNode;
    H->heapCost[H->indexOfLast] = keyValue;
    thisNode->heapIndex = H->indexOfLast;
    bubbleUp(H, H->indexOfLast);     
    thisNode->inHeap = true;
  }
}

// returns a pointer to the node that is on the top of the heap
nodeT* topHeap(sbbsHeap* H)
{
  if(H->indexOfLast >= 0)
  {
    return H->heapNode[0];
  }
  return NULL;
}


// removes the top valued node from the heap and returns a pointer to it 
nodeT* popHeap(sbbsHeap* H)
{

 //log_message_from_sim("pop");

  nodeT* oldTopNode = H->heapNode[0];
  H->heapNode[0] = H->heapNode[H->indexOfLast];
  H->heapCost[0] = H->heapCost[H->indexOfLast];
  H->heapNode[0]->heapIndex = 0;
  H->heapNode[H->indexOfLast] = NULL;
  H->heapCost[H->indexOfLast] = getInfinity();
  H->indexOfLast--;
  H->parentOfLast = (H->indexOfLast-1)/2;
  bubbleDown(H, 0);
  oldTopNode->inHeap = false;
  oldTopNode->heapIndex = -1;
  return oldTopNode;
}


// removes the particular node from the heap
// (even if that node is internal to the heap)
// and then rebalances the heap, returns a pointer
// to the node that has been removed
nodeT* removeNodeFromHeap(sbbsHeap* H, nodeT* thisNode)
{

  //log_message_from_sim("remove");
  if(!thisNode->inHeap)
  {
    return NULL;
  }

  int ind = thisNode->heapIndex;

  H->heapNode[ind] = H->heapNode[H->indexOfLast];
  H->heapCost[ind] = H->heapCost[H->indexOfLast];
  H->heapNode[ind]->heapIndex = ind;

  H->heapNode[H->indexOfLast] = NULL;
  H->heapCost[H->indexOfLast] = getInfinity();
  H->indexOfLast--;
  H->parentOfLast = (H->indexOfLast-1)/2;

  if(ind <= H->indexOfLast) 
  {  
    ind = bubbleUp(H, ind);
    bubbleDown(H, ind);
  }
  
  thisNode->inHeap = false;
  thisNode->heapIndex = -1;
  return thisNode;
}


// updates the position of the particular node within the heap
// to reflect the new key value
// (even if that node is internal to the heap)
// and then rebalances the heap)
// NOTE: this will insert the node if it is not in the heap already 
void updateNodeInHeap(sbbsHeap* H, nodeT* thisNode, double keyValue)
{
  // we'll just do the easy way
  if(thisNode->inHeap)
  {
    removeNodeFromHeap(H, thisNode);
  }
  addToHeap(H, thisNode, keyValue);
}



// removes the *last* node in the heap. Note this is not necessarily
// the node with the max cost, however, it is something that can be
// done in O(1) time while still leaving the heap sorted. It is useful
// for emptying the heap node by node in order to do somthing else, in
// the particuar case that we do not care about node order.
nodeT* removeNodeFromHeapBottom(sbbsHeap* H)
{
  //log_message_from_sim("remove bottom");

  if(H->indexOfLast < 0)
  {
    return NULL;
  }
  nodeT* thisNode = H->heapNode[H->indexOfLast];
          
  H->heapNode[H->indexOfLast] = NULL;
  H->heapCost[H->indexOfLast] = getInfinity();
  H->indexOfLast--;
  H->parentOfLast = (H->indexOfLast-1)/2;

  thisNode->inHeap = false;
  thisNode->heapIndex = -1;
  
  return thisNode;
}



// retuns true if the heap is emptuy
bool isEmpty(sbbsHeap* H)
{
  if(H->indexOfLast >= 0)
  {
    return false;
  }
  return true;
}

/*
// prints the heap values on the command line
// note this will break if T does not have field "id"
void printHeap(sbbsHeap* H)
{
  mexPrintf("heap costs:\n");

  int i,p; 
  char ch[10];
  for(i = 0, p = 0; i <= H->indexOfLast; i++)
  {    
    mexPrintf("%f ", H->heapCost[i]);
    if(i == p)
    {
       mexPrintf("\n");
       p = p+2^p;
    }
  }
  mexPrintf("\n\n");
  

  mexPrintf("heap node ids:\n");
  for(i = 0, p = 0; i <= H->indexOfLast; i++)
  {    
    mexPrintf("(%d) ", H->heapNode[i]->id);
    if(i == p)
    {
       mexPrintf("\n");
       p = p+2^p;
    }
  }
  mexPrintf("\n\n");
}
*/

// returns 1 if heap is good, 0 if bad, also prints a command line message
int checkHeap(sbbsHeap* H)
{
  int i;
  for(i = 0; i <= H->indexOfLast; i++)
  {
    if(H->heapCost[i] < H->heapCost[(i-1)/2] || H->heapNode[i]->heapIndex != i)
    {
      //mexPrintf("There is a problem with the heap \n");
      getchar();
      return false;
    }
  } 
  //mexPrintf("The heap is OK \n");
  return true;  
}


#endif //__SBBSHEAP_H__

