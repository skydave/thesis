/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once
#include <map>
#include <algorithm>

#include "dk/Mesh.h"

using namespace dk;


// used for singularvalue- and eigenvalue-decomposition
#include "nr3/nr3_util.h"



//
//
//
class SurfaceCrackGenerator
{
public:
	struct Element;

	//
	// correspondeces to the vertices of the input mesh
	//
	struct Node
	{
		Node();                                       // constructor
		void                                               registerElement( Element *e );  // adds the given element to the elementring list
		void                                             unRegisterElement( Element *e );  // remvoes the given element to the elementring list
		bool                                                                isDesolate();  // returns true if no element references this node
		int             getPlaneSide( const math::Vec3f &normal, const float &distance );  // returns 1 if the node lies on the left side of the given plane (nodeplanedistance < 0.0f) or 2 otherwise - returns 0 if point lies on the plane
		void                                    doEigenDecompositionOfSeperationTensor();  // this method will do a eigenvalue decomposition of the seperationtensor and stores the maximum eigenvalue and its vector

		math::Vec3f                                                             position;
		math::Vec3f                                                               normal;
		math::Color                                                                color;

		std::vector<Element *>                                               elementRing;  // references to all elements which index into this node

		math::Vec3f                                               unbalancedTensileForce;  // superposition of all tensile forces acting on that node from all surrounding elements
		math::Vec3f                                           unbalancedCompressiveForce;  // superposition of all compressive forces acting on that node from all surrounding elements

		std::vector<math::Vec3f>                                           tensileForces;  // vector of all tensile forces from all neighbouring elements
		std::vector<math::Vec3f>                                       compressiveForces;  // vector of all compressive forces from all neighbouring elements

		math::Matrix33f                                                 seperationTensor;  // this is a tensor which balances tensile and compressive forces

		float                                                              maxEigenvalue;  // maximum eigenvalue of the seperationtensor for this node
		math::Vec3f                                            maxEigenvaluesEigenvector;  // to the maximum eigenvalue corresponding eigenvector - defining the crack plane on failure

		math::Vec3f                                                  virtualDisplacement;  // displacement of the node which is caused by the stresses from surrounding elements

		math::Matrix33f                                               globalStressTensor;  // global stress tensor directly from the grid
		float                                                        maxGlobalEigenvalue;
	};

	//
	// We use the winged edge datastructure to ease mesh edit operations
	//
	struct Edge
	{
		Edge( Node *_n1, Node *_n2 );                                                                              // constructor

		void                                                                       registerElement( Element *e );  // assigns the given element to the left or right wing of the edge (dependand on which wing is 0)
		void                                                                     unregisterElement( Element *e );  // sets either the left or right reference to zero when it references the given element
		bool intersectsPlane( const math::Vec3f &normal, const float &distance, math::Vec3f &intersectionPoint );  // convinience function to compute the intersection of the edge with a given plane
		Node                                                                            *getOtherNode( Node *n );  // convinience function to get the other node of the two nodes which are referenced by the edge
		bool                                                                                        isDesolate();  // returns true if no elements references this edge
		bool                                                                                 contains( Node *n );  // returns true if the edge contains the given node
		bool                                                                                    isBoundaryEdge();  // returns true if the edge has only one element reference instead of two

		Node                                                                                                 *n1;  // Edge is made up of 2 nodes
		Node                                                                                                 *n2;
		Element                                                                                            *left;  // left neighbour - if it is 0, the edge is a borderedge
		Element                                                                                           *right;  // right neighbour - if it is 0, the edge is a borderedge
	};

	//
	// correspondences to the triangles of the input mesh
	//
	struct Element
	{
		union
		{
			struct
			{
				Node *n0, *n1, *n2;
			};
			Node *n[3];
		};
		union
		{
			struct
			{
				Edge *e1, *e2, *e3;
			};
			Edge *e[3];
		};

		Element();                                                                                          // constructor
		void                                                                           computeProperties(); // computes normal, area, etc...
		math::Vec3f  convertFromLocalToGlobalSpace( const math::Vec2f &localSpace, bool isVector = false ); // transforms the given coordinate in local 2d space into the global 3d space in which the triangle/element lies
		math::Vec2f convertFromGlobalToLocalSpace( const math::Vec3f &globalSpace, bool isVector = false ); // inverse operation of the above
		void                                                             applySigmaFrom( Element *parent ); // This method will take the stress of the parent element and applies it to this element.
		void                                       applyTensileAndCompressiveForcesToLocalNode( size_t i ); // computes and adds f_plus and f_minus to the list of tensile and compressive forces of the node specified with the local index within the element
		void                                                   capSingularValueOfBeta( float svThreshold ); // this will make sure that the singular value of beta will be kept to a certain maximum
		void                                                                 doEigenDecompositionOfSigma(); // this will compute the eigenvalues and eigenvectors of sigma and store the result in the local vectors
		math::Matrix33f                                   computeStrainDerivative( size_t node, size_t r ); // computes the change of the strain tensor of the current element in respect to the vector component r of the node-th node of the element
		Node                                                           *getOtherNode( Node *n1, Node *n2 ); // returns the third node which neither equals n1 nor n2
		Node                                                                   *getOtherNode( Edge *edge ); // returns the third node which neither equals n1 nor n2
		bool                                                                        contains( Node *node ); // returns true if one of the element nodes equals the given node n
		size_t                                                               getNodesLocalIndex( Node *n ); // returns the local index of the node within the element

		math::Vec3f                                                                                 center; // barycentric center position of the triangle
		math::Vec3f                                                                                 normal;
		math::Vec3f                                                                                      u; // these 2 vectors (u and v) define the local 2dimensional coordinate
		math::Vec3f                                                                                      v; // frame of the element (although they are 3d vectors)

		math::Matrix22f                                                                              sigma; // stresses in the triangle coordinate system (plane->2d stresses)
		std::vector< float >                                                             sigma_eigenValues; // eigenvalues of sigma
		std::vector< math::Vec2f >                                                      sigma_eigenVectors; // eigenvectors of sigma

		math::Matrix33f                                                                               beta; // barycentric coordinate matrix -> base vectors are the position of the 3 nodes of the element in the local 2d cooridnate frame
																											// so beta transforms local coordinates into barycentric coordinates

		float                                                                                         area; // the area of the element


		math::Matrix22f                                                                         sigma_plus; // tensile stresses
		math::Matrix22f                                                                        sigma_minus; // compressive stresses
	};


	SurfaceCrackGenerator( Mesh *mesh );                                                                       // constructor which takes a mesh as input
	Mesh                                                                                           *getMesh(); // creates and returns a mesh from the current topology



	void                                                                     applyUniformShrinkage( float c );
	void                                                    applyDirectionStress( float x, float y, float z );
	void                                                                                stressesInitialized(); // call this method if the stress tensors of all elements have been initialized
	void                                                                                   performIteration(); // does one iteration step of the algorithm
	void                                                                                  performRelaxation(); // does the stress relaxation step
	void                                                                        computeVirtualDisplacements(); // computes the virtual displacements of each node from the stresses of its surrounding elements
	float                                                                                 getRelaxationRate(); // returns the rate at which the stress relaxes in one relaxation-pass
	void                                                            setRelaxationRate( float relaxationRate ); // sets the rate at which the stress relaxes in one relaxation-pass
	size_t                                                                               getRelaxIterations(); // returns the number of iterations for the relaxationstep
	void                                                         setRelaxIterations( size_t relaxIterations ); // sets the number of iterations for the relaxationstep
	float                                                                              getMaterialToughness(); // returns the strength of crack-resistance
	void                                                      setMaterialToughness( float materialToughness ); // sets the strength of crack-resistance
	size_t                                                                            getCracksPerIteration(); // returns the number of cracks created per iteration
	void                                                   setCracksPerIteration( size_t cracksPerIteration ); // sets the number of cracks created per iteration
	float                                                                               getCrackPropagation(); // returns the crackpropagation parameter
	void                                                        setCrackPropagation( float crackPropagation ); // sets the crackpropagation parameter


//private:
	struct SplitCandidate
	{
		Edge                    *edge; // edge which has to be split
		Element              *element; // the element, whose edge has to be split
		math::Vec3f intersectionPoint; // the position where the edge would be split
	};


	Node                                                           *createNode( const math::Vec3f &position ); // creates a Node and adds it to the node list with the given world position
	void                                                                                removeNode( Node *n ); // removes given node from the node list
	Element                *createElement( const size_t &index0, const size_t &index1, const size_t &index2 ); // creates a Element and adds it to the element list with the given node indices
	Element                                   *createElement( Node *n0, Node *n1, Node *n2, bool capsv=true ); // creates a Element and adds it to the element list with the given node indices and caps singular values if told to do
	Element     *createElement( Node *n0, Node *n1, Node *n2, Edge *e0, Edge *e1, Edge *e2, bool capsv=true ); // creates a Element and adds it to the element list, the 3 edges which form the element are given in addition to speed up tghe process from given edges
	Edge                                                                    *createEdge( Node *n1, Node *n2 );
	Edge                                                                      *findEdge( Node *n1, Node *n2 ); // looks for an edge between the given nodes - returns 0 if it could not be found
	Node *                                          splitEdge( Edge *edge, const math::Vec3f &splitPosition ); // splits the given edge into 2 and returns the splitnode which was created to seperate the edge
	void                                                                             removeEdge( Edge *edge ); // removes edge from the list of edges
	void                                                                    removeElement( Element *element ); // removes element from the list of elements
	void                        replaceNodeWithinElement( Element *e, Node *originalNode, Node *replacement ); // replaces all occurances of the original node within element with the replacementnode and changes the edges accordingly
	void                                                                   computeSeperationTensor( Node *n ); // computes the seperation tensor from tensile and compressive forces

	math::Matrix22f                                                               m( const math::Vec2f &vec ); // this method converts a vector into a tensor by multiplying the vector with itsself using the outer-product and finally dividing it by the length of the vector
	math::Matrix33f                                                               m( const math::Vec3f &vec ); // this method converts a vector into a tensor by multiplying the vector with itsself using the outer-product and finally dividing it by the length of the vector

	std::vector<Node*>                                                                                  nodes;
	std::vector<Element*>                                                                            elements;
	std::vector<Edge*>                                                                                  edges;

	std::map< float, Node *, std::greater<float> >                                                      queue; // priority list of all nodes sorted by the strengh of their failure criteria

	size_t                                                                                     iterationCount; // tracks the number of iterations

	// parameters which influence the cracking
	float                                                                                   materialToughness;
	float                                                                                    crackPropagation; // the alpha parameter in the phd thesis controls how much cracks propagate from the cracktips
	float                                                                                      relaxationRate;
	size_t                                                                                    relaxIterations;
	size_t                                                                                 cracksPerIteration; // defines how many cracks are created per iteration
	float                                                                                         svThreshold; // the singular value threshold for the beta matrices (see chapter "avoiding numerical instabilities")


	// temp
	float temp;
};