/*---------------------------------------------------------------------

kdtree implementation for faster n-nearest-neighbour search

----------------------------------------------------------------------*/
#pragma once

#include <vector>
#include "Particle.h"






///
/// \brief spatial optimization structure
///
class kdTree
{
public:
	///
	/// \brief the kdtree is made of TreeNodes which organize the n-dimensional data
	/// in a binary tree
	///
	struct kdTreeNode
	{
		//
		// constructor
		//
		kdTreeNode( unsigned int _splitDimension, Particle *_element ) : splitDimension( _splitDimension ), element( _element ), left(0), right(0), parent(0)
		{
		}

		//
		// destructor
		//
		~kdTreeNode()
		{
			// delete sub-branches
			if( left )
				delete left;
			if( right )
				delete right;
		}

		//
		//
		// return: the number of neighbours within range
		void findNeighboursWithinRange( const math::Vec3f &position, float searchDistanceSquared, std::vector<Particle *> &searchResult )
		{
			// compute distance of current tree node to the targetpoint in the current splitDimension
			float dimensionDistance = element->position[ splitDimension ] - position[ splitDimension ];

			// the current kdTreeNode splits the space in 2 halfes on the current dimension
			// from the sign of the distance from the searchCenter to the current kdTreeNode in the current dimension
			// we can tell on which half the searchCenter lies
			if( dimensionDistance > 0.0f )
			{
				// we know that the searchCenter lies within the left side of the splitted space
				// so we should check the left side in any case
				if( left )
					left->findNeighboursWithinRange( position, searchDistanceSquared, searchResult );

				// we can omit checking the right side if the searchCenter lies so far within the left
				// halve that the searchRadius doesnt reach the other halve
				if( dimensionDistance*dimensionDistance <= searchDistanceSquared )
				{
					// the searchRadius touches the other halve, so we have to check that too
					if( right )
						right->findNeighboursWithinRange( position, searchDistanceSquared, searchResult );

					// we have to check wether the current kdTreeNode is within the search-distance
					// when the dimensionDistance touches the other halve and therefore touches
					// the current kdTreeNode (because it lies directly between the 2 halves)
					// so do a full euclidian distance check
					if( (element->position - position).getSquaredLength() < searchDistanceSquared )
						// the current kdTreeNode lies within the given searchRadius
						searchResult.push_back( element );
				}
			}else
			{
				// we know that the searchCenter lies within the right side of the splitted space
				// so we should check the right side in any case
				if( right )
					right->findNeighboursWithinRange( position, searchDistanceSquared, searchResult );

				// we can omit checking the left side if the searchCenter lies so far within the right
				// halve that the searchRadius doesnt reach the other halve
				if( dimensionDistance*dimensionDistance <= searchDistanceSquared )
				{
					// the searchRadius touches the other halve, so we have to check that too
					if( left )
						left->findNeighboursWithinRange( position, searchDistanceSquared, searchResult );

					// we have to check wether the current kdTreeNode is within the search-distance
					// when the dimensionDistance touches the other halve and therefore touches
					// the current kdTreeNode (because it lies directly between the 2 halves)
					// so do a full euclidian distance check
					if( (element->position - position).getSquaredLength() < searchDistanceSquared )
						// the current kdTreeNode lies within the given searchRadius
						searchResult.push_back( element );
				}
			}
		}

		kdTreeNode                  *left;  // left sub-branch
		kdTreeNode                 *right;  // right sub-branch
		kdTreeNode                *parent;  // reference to the parent node

		unsigned int       splitDimension;  // tells on which dimension the space is partitioned
		Particle                 *element;  // reference to the given data
	};

	///
	/// This method will insert the given element into the tree structure
	///
	void insert( Particle *element )
	{
		// we have a new element somewhere
		// first we have to find the node within the tree which will
		// have the new element as (left or right)child
		kdTreeNode* parent = findParent( element );
		
		//if(equal(p, parent->x, SD))
		//	return NULL ;

		if( parent == 0 )
		{
			// no root node yet
			root = new kdTreeNode( 0, element );
			return;
		}

		// we have the parent - create a new kdTreeNode representing the elemnt
		kdTreeNode* newNode = new kdTreeNode( parent->splitDimension + 1 < 3 ? parent->splitDimension + 1 : 0, element );

		// store the parent of our new node
		newNode->parent = parent;

		// hook the new node into the tree by referencing it from the parent
		// we have to decide on which side of the partitioned space the new node lives
		if( element->position[ parent->splitDimension ] > parent->element->position[ parent->splitDimension ] )
		{
			parent->right = newNode;
		}
		else
		{
			parent->left = newNode;
		}
	}

	//
	// this method will look for the treenode which divides the space in which
	// the given element lies
	//
	kdTreeNode *findParent( Particle *element )
	{
		kdTreeNode* parent = 0;

		kdTreeNode* next = root;

		unsigned int splitDimension;

		while( next )
		{
			splitDimension = next->splitDimension;

			parent = next;

			if( element->position[ splitDimension ] > next->element->position[ splitDimension ] )
				next = next->right;
			else
				next = next->left;
		}

		return parent;
	}

	//
	//
	// return: the number of neighbours within range
	unsigned int findNeighboursWithinRange( const math::Vec3f &position, float searchDistanceSquared, std::vector<Particle *> &searchResult )
	{
		if( root )
			root->findNeighboursWithinRange( position, searchDistanceSquared, searchResult );

		return 0;
	}

	//
	// deletes all created nodes
	//
	void clear()
	{
		if( root )
			delete root;

		root = 0;
	}


	//
	// constructor
	//
	kdTree()
	{
		root = 0;
	}

	//
	// destructor
	//
	~kdTree()
	{
		clear();
	}

private:
	kdTreeNode                   *root;       // root of the tree
};