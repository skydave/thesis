#include "SurfaceCrackGenerator.h"





//
// constructor which takes a mesh as input
//
SurfaceCrackGenerator::SurfaceCrackGenerator( Mesh *mesh )
{
	mesh->computeVertexIndicees();
	mesh->computeNormals();

	// convert the mesh into the local problem domain
	// create a node for each vertex
	for( std::vector<Mesh::Vertex *>::iterator it=mesh->vertices.begin(); it != mesh->vertices.end(); ++it )
	{
		Mesh::Vertex *vert = *it;
		Node *n = createNode(vert->position);
		n->normal = vert->normal;
	}

	/*
	// create a element for each triangle
	for( std::vector<Mesh::Triangle *>::iterator it=mesh->triangles.begin(); it != mesh->triangles.end(); ++it )
	{
		Mesh::Triangle *tri = *it;
		Element *e = createElement( tri->v[0]->index,tri->v[1]->index, tri->v[2]->index );
	}
	*/





	std::map< Node *, std::vector<Edge *> > edgeHash; // stores all incoming and outgoing edges for each vertex

	for( std::vector<Mesh::Triangle *>::iterator it=mesh->triangles.begin(); it != mesh->triangles.end(); ++it )
	{
		Mesh::Triangle *tri = *it;


		// get vertices of the triangle
		Node *n0 = nodes[tri->v[0]->index];
		Node *n1 = nodes[tri->v[1]->index];
		Node *n2 = nodes[tri->v[2]->index];

		// edges which will build the mesh
		Edge *e0, *e1, *e2;

		e0 = e1 = e2 = 0;

		// look for edge n0->n1 and n2->n0
		std::vector<Edge *> &n0_edges = edgeHash[n0];

		for( std::vector<Edge *>::iterator eit = n0_edges.begin(); eit != n0_edges.end(); ++eit )
		{
			Edge *e = *eit;
			if( e->getOtherNode( n0 ) == n1 )
			{
				e0 = e;
			}else
			if( e->getOtherNode( n0 ) == n2 )
			{
				e2 = e;
			}
		}

		// edge not found?
		if( !e0 )
		{
			// create it
			e0 = createEdge( n0, n1 );
			// register edge with the hash
			edgeHash[n0].push_back( e0 );
			edgeHash[n1].push_back( e0 );
		}

		// edge not found?
		if( !e2 )
		{
			// create it
			e2 = createEdge( n2, n0 );
			// register edge with the hash
			edgeHash[n2].push_back( e2 );
			edgeHash[n0].push_back( e2 );
		}

		// look for edge n1->n2
		std::vector<Edge *> &n1_edges = edgeHash[n1];

		for( std::vector<Edge *>::iterator eit = n1_edges.begin(); eit != n1_edges.end(); ++eit )
		{
			Edge *e = *eit;
			if( e->getOtherNode( n1 ) == n2 )
			{
				e1 = e;
				break;
			}
		}

		// edge not found?
		if( !e1 )
		{
			// create it
			e1 = createEdge( n1, n2 );
			// register edge with the hash
			edgeHash[n1].push_back( e1 );
			edgeHash[n2].push_back( e1 );
		}

		Element *e = createElement( n0, n1, n2, e0, e1, e2, false );
	}






	// initialize stress tensors of all elements
	for( std::vector<Element*>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
	{
		Element *e = *eit;
		e->sigma = math::Matrix22f::Zero();
	}

	// compute svThreshold and eigenvalues/vectors of sigma
	//stressesInitialized();


	// initialize members
	setMaterialToughness( 0.05f );

	setCrackPropagation( 0.75f );

	setCracksPerIteration( 3 );

	setRelaxationRate( 0.005f );
	setRelaxIterations( 7 );

	// shrinkageRate

	iterationCount = 0;	
}

//
// creates and returns a mesh from the current topology
//
Mesh *SurfaceCrackGenerator::getMesh()
{
	std::map<Node *, int>          indicees; // the index into the vertexvector for each vertex
	std::vector<math::Vec3f> vertexPosition; // position for each vertex
	std::vector<int>       triangleIndicees; // 3 indicees into the vertexvector for each triangle


	vertexPosition.reserve( nodes.size() );
	triangleIndicees.reserve( elements.size()*3 );

	int count = 0;
	for( std::vector<Node *>::iterator it = nodes.begin(); it != nodes.end(); ++it )
	{
		Node *n = *it;
		vertexPosition.push_back( n->position );
		indicees[ n ] = count++;
	}

	for( std::vector<Element *>::iterator it = elements.begin(); it != elements.end(); ++it )
	{
		Element *e = *it;
		triangleIndicees.push_back( indicees[e->n0] );
		triangleIndicees.push_back( indicees[e->n1] );
		triangleIndicees.push_back( indicees[e->n2] );
	}

	return new Mesh( vertexPosition, triangleIndicees );
}

void SurfaceCrackGenerator::applyUniformShrinkage( float c )
{
	// modify all stress tensors of all elements
	for( std::vector<Element*>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
	{
		Element *e = *eit;

		// uniform shrinkage in all directions
		e->sigma._11 += c;
		e->sigma._22 += c;
	}
}

void SurfaceCrackGenerator::applyDirectionStress( float x, float y, float z )
{
	// modify all stress tensors of all elements
	for( std::vector<Element*>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
	{
		Element *e = *eit;

		e->sigma = math::Matrix22f::Zero();

		// unidirectional tension
		math::Vec3f globalDirection( x, y, z );
		//if( (e->center.x > 3.0f) && (e->center.x < 7.0f))
		{
			math::Vec2f localDirection = e->convertFromGlobalToLocalSpace(globalDirection, true);
			e->sigma += math::outerProduct( localDirection, localDirection );
		}
	}
}



//
// call this method if the stress tensors of all elements have been initialized
//
void SurfaceCrackGenerator::stressesInitialized()
{

	// compute svThreshold and do eigendecomposition of sigma
	svThreshold = 0.0f;
	for( std::vector<Element*>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
	{
		Element *e = *eit;

		// compute eigenvectors and eigenvalues
		e->doEigenDecompositionOfSigma();

		// singular value decomposition of the beta-matrix of each element to compute
		// the maximum threshold for beta-singular values (see "avoiding numerical instabilities" in the thesis)
		MatDoub beta( 3, 3 );
		for( int i=0;i<3; ++i )
			for( int j=0;j<3; ++j )
				beta[i][j]=e->beta.m[i][j];

		SVD betasvd( beta );

		//printf( "singular values of beta: %f   %f   %f\n", betasvd.w[0], betasvd.w[1], betasvd.w[2] );

		svThreshold += (float)std::max<double>( betasvd.w[0], std::max<double>( betasvd.w[1], betasvd.w[2] ) );
		
	}

	svThreshold = 8.0f*( svThreshold / (float)elements.size() );
}


//
// does one iteration step of the algorithm
//
void SurfaceCrackGenerator::performIteration()
{
	// preparation ---------------------------------------------------------------------------------------

	// clear the cracknode queue
	queue.clear();

	// reinitialize all nodes
	for( std::vector<Node*>::iterator nit = nodes.begin(); nit != nodes.end(); ++nit )
	{
		Node *n = *nit;

		// reset tensile and compressive forces
		n->unbalancedTensileForce = math::Vec3f();
		n->unbalancedCompressiveForce = math::Vec3f();

		n->tensileForces.clear();
		n->compressiveForces.clear();
	}


	// decompose sigma into sigma_plus and sigma_minus through eigenvector decomposition ----------------
	// and compute tensile and compressive forces exerted on each node from each element

	// iterate over all elements
	for( std::vector<Element*>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
	{
		Element *e = *eit;

		// compute sigma_plus and sigma_minus from sigma
		e->sigma_plus = math::Matrix22f::Zero();
		e->sigma_minus = math::Matrix22f::Zero();

		// original
		e->sigma_plus += std::max<float>( 0, e->sigma_eigenValues[0] )*m(e->sigma_eigenVectors[0]);
		e->sigma_plus += std::max<float>( 0, e->sigma_eigenValues[1] )*m(e->sigma_eigenVectors[1]);

		e->sigma_minus += std::min<float>( 0, e->sigma_eigenValues[0] )*m(e->sigma_eigenVectors[0]);
		e->sigma_minus += std::min<float>( 0, e->sigma_eigenValues[1] )*m(e->sigma_eigenVectors[1]);


		// compute the tensile and compressive forces which are exerted by this element onto each
		// surrounding node

		// for each node of the element
		for( size_t i=0; i<3; ++i )
			e->applyTensileAndCompressiveForcesToLocalNode( i );
	}

	// compute seperationtensor for each node
	// iterate over all nodes
	for( std::vector<Node*>::iterator nit = nodes.begin(); nit != nodes.end(); ++nit )
	{
		Node *n = *nit;

		// we take onle nodes with more than one elements within the elementring into account
		if( n->elementRing.size() < 2 )
			continue;

		// compute the seperation tensor of the current node
		computeSeperationTensor( n );
		// find maximum eigenvalue and its vector
		n->doEigenDecompositionOfSeperationTensor();

		// does the material fail at the node ?
		if( fabs(n->maxEigenvalue) > materialToughness )
			// yes, add the node on the priority-queue of failed nodes
			queue[n->maxEigenvalue] = n;
	}

	// done the queue now holds all nodes which stress situation fails the material toughness
	// and therefore are candidates for forming crack


	// crack the next node  ---------------------------------------------------------------------
	size_t cracksCreated = 0;
	// if there is a node which fails
	while( !queue.empty() && (cracksCreated++<cracksPerIteration) )
	{
		// we will crack the topmost node
		Node *cn = queue.begin()->second;

		// TEMP:
		//temp = cn->maxEigenvalue;
		//return;

		// remove the node from the priority queue
		queue.erase( queue.begin() );

		// normal of the crackplane
		math::Vec3f normal = math::normalize( cn->maxEigenvaluesEigenvector );
		// distance of the crackplane from the origin
		float distance = -math::dotProduct( normal, cn->position );

		// list of potential splits (element + splitedge and the intersection point on that edge)
		std::vector<SplitCandidate>                  splits;

		// this is a list of node which lie on the tip of the newly created crack
		// these are used for crack propagation
		std::vector<Node *>                   crackTipNodes;

		// intersect the plane given by the maximum eigenvaluevector with each surrounding element
		for( std::vector<Element *>::iterator eit=cn->elementRing.begin(); eit!=cn->elementRing.end(); ++eit )
		{
			Element *e = *eit;

			// iterate over all edges of the element and intersect the edge which does not contain
			// the cracknode n with the crackplane. All edges which contain n will not intersect with
			// the crackplane because they either start on the plane or end on the plance since the
			// cracknode lies directly on the plane.
			// for each edge
			for( size_t i=0; i<3; ++i )
			{
				// if the current edge does not contain the cracknode
				if( !e->e[i]->contains(cn) )
				{
					// intersect the edge with the crackplane
					math::Vec3f intersectionPoint;

					// if the ray intersects edge #i of the element
					if( e->e[i]->intersectsPlane( normal, distance, intersectionPoint ) )
					{
						// schedule this edge for splitting
						SplitCandidate split;
						split.edge = e->e[i];
						split.element = e;
						split.intersectionPoint = intersectionPoint;
						splits.push_back( split );
					}
				}
			}
		}

		// now we have at maximum 2 at minimum 1 element which intersects with the crackplane
		// for all other elements of the nodes elementring we can easily decide on which side
		// of the crackplane they lie and therefore can replace the cracknode cn with the
		// appropriate node of the side of the crackplane to which the element belongs to

		// the cracknode has to be split into 2 new nodes - one for each side of the crackplane
		Node *sn_left = createNode( cn->position );
		Node *sn_right = createNode( cn->position );


		// copy the elementring of the node since it will be changed during the following loop when
		// edges get removed
		std::vector<Element*> elementRing( cn->elementRing.begin(), cn->elementRing.end() );

		// iterate over all cracknode-neighbouring elements
		for( std::vector<Element*>::iterator eit = elementRing.begin(); eit!=elementRing.end(); ++eit )
		{
			// the element in question
			Element *e = *eit;

			// if the current element of the elementring belongs to an element which intersects with
			// the crackplane then we will skip it since it has to be treated more special because
			// it can not easily be decided on which crackplane it lies
			bool isSplitCandidate=false;
			for( std::vector<SurfaceCrackGenerator::SplitCandidate>::iterator sit = splits.begin(); sit != splits.end(); ++sit )
				if( (*sit).element == e )
				{
					isSplitCandidate = true;
					break;
				}
			if( isSplitCandidate )
				continue;;

			// now we can be sure that only elements which dont intersect with the crackplane will
			// be tested

			// the side on which the element lies - 0 means uninitialized, 1 means left side, 2 means right side
			int side = 0;

			// the nodes which have to be tested
			std::vector<Node*> testNodes;

			// add all nodes except the cracknode to the list of nodes which have to be tested
			for( size_t i=0; i<3; ++i )
				if( e->n[i] != cn ) testNodes.push_back( e->n[i] );

			// test all testnodes, they should all lie on the same side of the crackplane
			for( std::vector<Node*>::iterator nit = testNodes.begin(); nit != testNodes.end(); ++nit )
			{
				 Node *testNode = *nit;

				 int thisSide = testNode->getPlaneSide( normal, distance );

				 // if side is already initialized, then...
				 if( side != 0 )
					 // ...make sure that the current point does not lie on the other side
					 if( thisSide != side )
					 {
						 printf( "error during splitting Node - Element of the elementring lies on both sides of the crackplane\n" );
					 }

				side = thisSide;
			}

			// we now know on which side of the crackplane the element lies
			if( !side )
				printf( "error : could not decide on which side of the crackplane the element lies\n" );

			// the references to the cracknode of all elements of the elementring have to be redirected
			if( side == 1 )
				replaceNodeWithinElement( e, cn, sn_left );
			if( side == 2 )
				replaceNodeWithinElement( e, cn, sn_right );
		} // iteration over all ringelements

		// now we have treated all elements of the nodes elementring which dont intersect with the
		// crackplane and replaced its references to the cracknode with references to the splitnode which
		// belongs to the side of the element

		// we know have to treat the elements which intersect with the crackplane
		// either we split it into 2 halves or we snap the splitpoint to one of the sideedges

		// now handle all elements which intersect with the crackplane
		while(!splits.empty())
		{
			// get the element which interesects with the crackplane
			Element *splitElement = splits.begin()->element;

			// check angles between vector (intersectionpoint-n->position) and (the sideedge)
			// edge snapping angle in degree
			float edgeSnap = math::degToRad( 15.0f );
			// crack-edge snapping angle in degree
			float crackEdgeSnap = math::degToRad( 25.0f );

			math::Vec3f splitVector = math::normalize( splits.begin()->intersectionPoint - cn->position );

			// minimum angle
			float minAngle = 9999999999999999.0f;
			Edge *minEdge = 0;

			// iterate over all edges of that element
			for( size_t i=0; i<3; ++i )
			{
				// if the current edge of the current ringelement contains the cracknode...
				if( splitElement->e[i]->contains( cn ) )
				{
					// ...then we will compute the angle between that edge and the edge which
					// would be created by the edgesplit
					float angle = acosf( math::dotProduct( math::normalize(splitElement->e[i]->getOtherNode(cn)->position - cn->position), splitVector ) );

					// we are looking for the closest angle
					if( fabsf(angle) < minAngle )
					{
						minAngle = angle;
						minEdge = splitElement->e[i];
					}
				}
			}

			
			// check if the new edge which would be created from cracking the current splitedge
			// is too close to any of the sideedges...
			if( minEdge &&                                                 // a minimumedge had to be found
					((minAngle<edgeSnap)||                                 // we snap to normal cracks with an angle of 15°
					((minAngle<crackEdgeSnap) && minEdge->isBoundaryEdge())// we snap to crack/boundary edges with an angle of 25° to avoid backcracking
					)
			  )
			{
				// the edge which would be created from splitting the splitedge
				// would be too close to the side edge, so we snap to the sideedge and
				// dont split

				// the node which lies on the edge to which we have snapped lies on the tip of the crack
				crackTipNodes.push_back( minEdge->getOtherNode(cn) );

				// decide on which side the element lies
				int side = splitElement->getOtherNode( minEdge )->getPlaneSide( normal, distance );

				// and handle it
				// the references to the cracknode of all elements of the elementring have to be redirected
				if( side == 0 )
					printf( "error : snapping: could not decide on which side of the crackplane the element lies\n" );
				if( side == 1 )
					replaceNodeWithinElement( splitElement, cn, sn_left );
				if( side == 2 )
					replaceNodeWithinElement( splitElement, cn, sn_right );
			}else
			{
				// perform the edgesplit
				Node *splitEdgeNode = splitEdge( splits.begin()->edge, splits.begin()->intersectionPoint );
				
				// lock for the edge which connects the cracknode with the splitedgenode...
				Edge *crackEdge = findEdge( cn, splitEdgeNode );

				// and replace the references to the cracknode with the references to the node of the
				// appropriate side
				if( crackEdge->left )
				{
					// decide on which side the element lies
					int side = crackEdge->left->getOtherNode( crackEdge )->getPlaneSide( normal, distance );

					// and handle it
					// the references to the cracknode of all elements of the elementring have to be redirected
					if( side == 0 )
						printf( "error : split: could not decide on which side of the crackplane the element lies\n" );
					if( side == 1 )
						replaceNodeWithinElement( crackEdge->left, cn, sn_left );
					if( side == 2 )
						replaceNodeWithinElement( crackEdge->left, cn, sn_right );
				}
				if( crackEdge->right )
				{
					// decide on which side the element lies
					int side = crackEdge->right->getOtherNode( crackEdge )->getPlaneSide( normal, distance );

					// and handle it
					// the references to the cracknode of all elements of the elementring have to be redirected
					if( side == 0 )
						printf( "error : split: could not decide on which side of the crackplane the element lies\n" );
					if( side == 1 )
						replaceNodeWithinElement( crackEdge->right, cn, sn_left );
					if( side == 2 )
						replaceNodeWithinElement( crackEdge->right, cn, sn_right );
				}

				// the node which results from spitting the edge lies on the tip of the cracknode
				crackTipNodes.push_back( splitEdgeNode );
			}

			// remove it from the list
			splits.erase( splits.begin() );
		};  // while(!splits.empty())



		// compute the seperation tensor of the newly created nodes from splitting the cracknode
		//for( std::vector<Element*>::iterator erit = sn_left->elementRing.begin(); erit!=sn_left->elementRing.end(); ++erit )
		//	(*erit)->applyTensileAndCompressiveForcesToLocalNode( (*erit)->getNodesLocalIndex(sn_left) );
		//computeSeperationTensor( sn_left );
		// if the material fails at the newly created node, we add it to the list
		//if( sn_left->maxEigenvalue > materialToughness )
			// yes, add the node on the priority-queue of failed nodes
			//queue[sn_left->maxEigenvalue] = sn_left;

		//for( std::vector<Element*>::iterator erit = sn_right->elementRing.begin(); erit!=sn_right->elementRing.end(); ++erit )
		//	(*erit)->applyTensileAndCompressiveForcesToLocalNode( (*erit)->getNodesLocalIndex(sn_right) );
		//computeSeperationTensor( sn_right );
		// if the material fails at the newly created node, we add it to the list
		//if( sn_right->maxEigenvalue > materialToughness )
			// yes, add the node on the priority-queue of failed nodes
			//queue[sn_right->maxEigenvalue] = sn_right;

		//
		// crack tip propagation -----------------------------------
		//
		// we modify the seperationtensor of the nodes on the cracktip to account for crackpropagation
		// see chapter 5.1 of Ibens phd thesis
		if( crackPropagation > 0.0f )
		{
			float residual = cn->maxEigenvalue - materialToughness;
			math::Matrix33f sepadd = crackPropagation*m(cn->maxEigenvaluesEigenvector)*residual;

			for( size_t i=0; i<crackTipNodes.size(); ++i )
			{
				Node *ctn = crackTipNodes[i];

				// remove the cracktipnode from the queue list (if it is contained)
				std::map<float, Node*, std::greater<float> >::iterator it;
				for( it = queue.begin(); it != queue.end(); ++it )
					if( it->second == ctn )
						break;
				if( it != queue.end() )
					queue.erase( it );

				// not sure: recompute the seperationtensors of the cracktipnodes and add 
				// ....

				// adapt seperation tensor for crackpropagation
				ctn->seperationTensor += sepadd;
				ctn->doEigenDecompositionOfSeperationTensor();

				// check for material failure and add the cracktip node to the queue with
				// the new failuer value
				if( ctn->maxEigenvalue > materialToughness )
					queue[ctn->maxEigenvalue] = ctn;			
			}
		}

		// now the cracknode should be desolated
		if( cn->isDesolate() )
			removeNode( cn );
		else
			printf( "error : original cracknode still referenced\n" );


	} // if cracknode queue is not empty -> !queue.empty()

	//
	// do stress relaxation ------------------------------------------------------------
	//
	for( size_t i=0; i<relaxIterations; ++i )
		performRelaxation();

	// increase iteration count
	++iterationCount;
}

//
// does the stress relaxation step
//
void SurfaceCrackGenerator::performRelaxation()
{
	// Relaxation is used to redistribute from areas of high stress to areas of lower stress
	// To perform relaxation we compute the virtual displacement of each node to determin the
	// change of the stressfield

	// first we compute each nodes virtual displacement from current stress-stituation
	computeVirtualDisplacements();


	// iterate over all elements again and update sigma of all elements depending on the virtual displacement of each node
	for( std::vector<Element*>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
	{
		Element *e = *eit;

		// build change of sigma for each node of the element and accumulate it onto sigma
		for( size_t i=0; i<3; ++i )
		{
			math::Matrix33f deltaSigma = math::Matrix33f::Zero();

			// each nodes component contributes a row for the strain derivative of the
			// current node
			for( int r=0; r<3; ++r )
				deltaSigma += e->computeStrainDerivative( i, r )*e->n[i]->virtualDisplacement[r]*relaxationRate;

			// accumulate change of sigma for each node onto the stress of the element
			// where we just leave out the third component
			e->sigma._11 += deltaSigma._11;
			e->sigma._12 += deltaSigma._12;
			e->sigma._21 += deltaSigma._21;
			e->sigma._22 += deltaSigma._22;
		}

		e->doEigenDecompositionOfSigma();
	}
}

//
// computes the virtual displacements of each node from the stresses of its surrounding elements
//
void SurfaceCrackGenerator::computeVirtualDisplacements()
{
	// To determine the displacement we compute the forces exerted on a node using eq 3.1 encapsulating
	// the effect of the stressfield on the node

	// compute nodal displacement
	for( std::vector<Node*>::iterator nit = nodes.begin(); nit != nodes.end(); ++nit )
	{
		Node *n = *nit;

		// reset virtual displacement
		n->virtualDisplacement = math::Vec3f( 0.0f, 0.0f, 0.0f );			
	}

	// iterate over all elements and compute contribution of each element on the virtual displacement of all
	// surrounding nodes
	for( std::vector<Element*>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
	{
		Element *e = *eit;

		// compute F which are the forces exerted by this element onto each
		// surrounding node

		// compute force acting on each node from the current element (after the equation 3.1 in the phd thesis of iben)
		for( size_t i=0; i<3; ++i )
		{
			math::Vec3f f( 0.0f, 0.0f, 0.0f );  // forces

			// for each node of the element
			for( size_t j=0; j<3; ++j )
			{
				float sum = 0.0f;

				for( size_t k=0; k<2; ++k )
					for( size_t l=0; l<2; ++l )
					{
						sum += e->beta.m[j][l]*e->beta.m[i][k]*e->sigma.m[k][l]; // correct version
					}

				f += e->n[j]->position*sum;
			}

			f *= -e->area;

			// add forces to the nodes virtual displacement
			e->n[i]->virtualDisplacement += f;
		}
	}
}

//
// computes the seperation tensor of given node from its tensile and compressive forces
//
void SurfaceCrackGenerator::computeSeperationTensor( Node *n )
{
	// compute superpositions of the tensile and compressive forces
	math::Matrix33f temp_tensile = math::Matrix33f::Zero();
	math::Matrix33f temp_compressive = math::Matrix33f::Zero();

	for( std::vector<math::Vec3f>::iterator fit = n->tensileForces.begin(); fit != n->tensileForces.end(); ++fit )
	{
		n->unbalancedTensileForce += *fit;
		temp_tensile += m( *fit );
	}
	for( std::vector<math::Vec3f>::iterator fit = n->compressiveForces.begin(); fit != n->compressiveForces.end(); ++fit )
	{
		n->unbalancedCompressiveForce += *fit;
		temp_compressive += m( *fit );
	}

	// compute seperation tensor
	n->seperationTensor = 0.5f*( -m(n->unbalancedTensileForce) + temp_tensile + m(n->unbalancedCompressiveForce) - temp_compressive );

	// we use the alternative stress tensor given by the application
	//n->seperationTensor = n->globalStressTensor;
}


//
// this method converts a vector into a tensor by multiplying the
// vector with itsself using the outer-product and finally dividing it by the length of the vector
//
math::Matrix22f SurfaceCrackGenerator::m( const math::Vec2f &vec )
{
	// zero vector?
	if( (vec.x == 0.0f)&&(vec.y == 0.0f) )
		// return zero tensor
		return math::Matrix22f::Zero();

	float length = vec.getLength();

	math::Matrix22f result = math::Matrix22f::Zero();

	result._11 = vec.x*vec.x;
	result._21 = vec.x*vec.y;
	result._22 = vec.y*vec.y;
	result._12 = vec.y*vec.x;


	return result / length;
}

//
// splits the given edge into 2 and returns the splitnode which was created to seperate the edge
//
SurfaceCrackGenerator::Node *SurfaceCrackGenerator::splitEdge( Edge *edge, const math::Vec3f &splitPosition )
{
	// if the edge had an element on the left then it must be replaced with 2 new elements
	// same counts for the right side
	Element   *left = 0; // indicates whether the edge had a reference to a left element
	Element    leftCopy; // used to hold the values of the parent element after it has been removed by calling removeEdge
	Node *l1[3], *l2[3]; // nodes for the new sub-elements which appear from dividing the left element

	Element  *right = 0; // indicates whether the edge had a reference to a right element
	Element   rightCopy; // used to hold the values of the parent element after it has been removed by calling removeEdge
	Node *r1[3], *r2[3]; // nodes for the new sub-elements which appear from dividing the right element

	// add the new node
	Node *sn = this->createNode( splitPosition );

	// is there a element on the left side of the edge?
	if( edge->left )
	{
		left = edge->left;
		leftCopy = *edge->left;

		// edgeindex of the edge which has to be split
		int ei;
		// which edge has to be split?
		for( ei=0; ei<3; ++ei )
			if( left->e[ei] == edge )
				break;
		l1[0] = left->n[ei];
		l1[1] = sn;
		l1[2] = left->n[(ei+2)%3];
		l2[0] = sn;
		l2[1] = left->n[(ei+1)%3];
		l2[2] = left->n[(ei+2)%3];
	}

	// is there a element on the right side of the edge?
	if( edge->right )
	{
		right = edge->right;
		rightCopy = *edge->right;

		// edgeindex of the edge which has to be split
		int ei;
		// which edge has to be split?
		for( ei=0; ei<3; ++ei )
			if( right->e[ei] == edge )
				break;
		r1[0] = right->n[ei];
		r1[1] = sn;
		r1[2] = right->n[(ei+2)%3];
		r2[0] = sn;
		r2[1] = right->n[(ei+1)%3];
		r2[2] = right->n[(ei+2)%3];
	}

	// remove edge from list (which will remove the 2 wing elements also )
	removeEdge( edge );

	// add the new elements (beware that left and right point to invalid locations since these elements
	// have been removed by removeEdge)
	if( left )
	{
		Element *el1 = createElement( l1[0], l1[1], l1[2] );
		Element *el2 = createElement( l2[0], l2[1], l2[2] );

		// rotate the stress tensor into the local coordinate system of the new elements
		el1->applySigmaFrom( &leftCopy );
		el2->applySigmaFrom( &leftCopy );
	}
	if( right )
	{
		Element *er1 = createElement( r1[0], r1[1], r1[2] );
		Element *er2 = createElement( r2[0], r2[1], r2[2] );

		// rotate the stress tensor into the local coordinate system of the new elements
		er1->applySigmaFrom( &rightCopy );
		er2->applySigmaFrom( &rightCopy );
	}

	return sn;
}

//
// looks for an edge between the given nodes - returns 0 if it could not be found
//
SurfaceCrackGenerator::Edge *SurfaceCrackGenerator::findEdge( Node *n1, Node *n2 )
{
	for( std::vector<Edge *>::iterator eit = edges.begin(); eit != edges.end(); ++eit )
	{
		Edge *e = *eit;
		if( ((e->n1 == n1)&&(e->n2 == n2))||((e->n1 == n2)&&(e->n2 == n1)) )
			return e;
	}

	return 0;
}

//
// creates a new edge from 2 given nodes
//
SurfaceCrackGenerator::Edge *SurfaceCrackGenerator::createEdge( Node *n1, Node *n2 )
{
	Edge *edge = new Edge( n1, n2 );
	// store edge
	edges.push_back( edge );
	return edge;
}

//
// removes edge from the list of edges
//
void SurfaceCrackGenerator::removeEdge( Edge *edge )
{
	for( std::vector<Edge *>::iterator eit = edges.begin(); eit != edges.end(); ++eit )
		if( edge == *eit )
		{
			edges.erase( eit );

			// remove the elements which neighboured the edge
			if( edge->left )
				removeElement( edge->left );
			if( edge->right )
				removeElement( edge->right );

			delete edge;			

			return;
		}
}

//
// creates a Node and adds it to the node list with the given world position
//
SurfaceCrackGenerator::Node *SurfaceCrackGenerator::createNode( const math::Vec3f &position )
{
	Node *n = new Node();
	n->position = position;
	nodes.push_back( n );
	return n;
}

//
// removes given node from the node list
//
void SurfaceCrackGenerator::removeNode( Node *n )
{
	nodes.erase( std::remove( nodes.begin(), nodes.end(), n ) );

	delete n;
}


//
// creates a Element and adds it to the element list with the given node indices
//
SurfaceCrackGenerator::Element *SurfaceCrackGenerator::createElement( const size_t &index0, const size_t &index1, const size_t &index2 )
{
	return createElement( nodes[index0], nodes[index1], nodes[index2], false );
}

//
// 
// Capsv tells whether the singular values of beta from the new element should eb capped to the svthreshold.
//
SurfaceCrackGenerator::Element *SurfaceCrackGenerator::createElement( Node *n0, Node *n1, Node *n2, bool capsv )
{
	Element *e = new Element();

	e->n0 = n0;
	e->n1 = n1;
	e->n2 = n2;

	// register the element with its ith node
	e->n0->registerElement( e );
	e->n1->registerElement( e );
	e->n2->registerElement( e );

	// create winged edge information
	Edge *edge = 0;

	edge = findEdge( e->n0, e->n1 );
	// if edge between n0 and n1 does not exist
	if( !edge )
		// create edge
		edge = createEdge( e->n0, e->n1 );
	edge->registerElement( e );
	e->e1 = edge;

	edge = findEdge( e->n1, e->n2 );
	// if edge between n1 and n2 does not exist
	if( !edge )
		// create edge
		edge = createEdge( e->n1, e->n2 );
	edge->registerElement( e );
	e->e2 = edge;


	edge = findEdge( e->n2, e->n0 );
	// if edge between n2 and n0 does not exist
	if( !edge )
		// create edge
		edge = createEdge( e->n2, e->n0 );
	edge->registerElement( e );
	e->e3 = edge;

	// compute normal, u, v, beta, center etc.
	e->computeProperties();



	// do the singular values of beta have to be capped to the maximum singular value threshold?
	// see "avoiding numerical instabilities" in the thesis for details
	if( capsv )
		e->capSingularValueOfBeta( svThreshold );




	elements.push_back( e );
	return e;
}

//
// creates a Element and adds it to the element list, the 3 edges
// which form the element are given in addition to speed up tghe process from given edges
// capsv tells whether the singular values of the beta matrix of the new element shall be
// kept to a certain threshold.
//
SurfaceCrackGenerator::Element *SurfaceCrackGenerator::createElement( Node *n0, Node *n1, Node *n2, Edge *e0, Edge *e1, Edge *e2, bool capsv )
{
	Element *e = new Element();


	e->n0 = n0;
	e->n1 = n1;
	e->n2 = n2;

	e->e1 = e0;
	e->e2 = e1;
	e->e3 = e2;

	// register the element with its ith node
	e->n0->registerElement( e );
	e->n1->registerElement( e );
	e->n2->registerElement( e );

	for( int i=0; i<3; ++i )
		e->e[i]->registerElement( e );

	e->computeProperties();

	// do the singular values of beta have to be capped to the maximum singular value threshold?
	// see "avoiding numerical instabilities" in the thesis for details
	if( capsv )
		e->capSingularValueOfBeta( svThreshold );

	elements.push_back( e );
	return e;
}

//
// removes element from the list of elements
//
void SurfaceCrackGenerator::removeElement( Element *element )
{
	// remove elements from the list
	for( std::vector<Element *>::iterator eit = elements.begin(); eit != elements.end(); ++eit )
		if( element == *eit )
		{
			elements.erase( eit );

			for( size_t i=0; i<3; ++i )
			{
				// remove references to the element from neighbouring edges
				element->e[i]->unregisterElement( element );
				// remove the element from the elementring of neighbouring nodes
				element->n[i]->unRegisterElement( element );
			}

			delete element;

			return;
		}
}


//
// this method converts a vector into a tensor by multiplying the
// vector with itsself using the outer-product and finally dividing it by the length of the vector
//
math::Matrix33f SurfaceCrackGenerator::m( const math::Vec3f &vec )
{
	float length = vec.getLength();

	// zero vector?
	if( length < 0.000000001f )
		// return zero tensor
		return math::Matrix33f::Zero();
	
	math::Matrix33f result = math::Matrix33f::Zero();

	result._11 = vec.x*vec.x;
	result._12 = vec.y*vec.x;
	result._13 = vec.z*vec.x;

	result._21 = vec.x*vec.y;
	result._22 = vec.y*vec.y;
	result._23 = vec.z*vec.y;

	result._31 = vec.x*vec.z;
	result._32 = vec.y*vec.z;
	result._33 = vec.z*vec.z;

	return result / length;
}


//
// replaces all occurances of the original node within element
// with the replacementnode and changes the edges accordingly
//
void SurfaceCrackGenerator::replaceNodeWithinElement( Element *e, Node *originalNode, Node *replacement )
{
	for( size_t i=0; i<3; ++i )
	{
		// check the current node if it has to be replaced
		if( e->n[i] == originalNode )
		{
			// remove the element from the elementring of the original node
			e->n[i]->unRegisterElement( e );
			// replace the node
			e->n[i] = replacement;
			// add the element to the elementring of the new node
			e->n[i]->registerElement( e );
		}

		// contains the current edge the original node?
		if( (e->e[i]->n1 == originalNode)||(e->e[i]->n2 == originalNode) )
		{
			// yes, replace the edge with a new one
			Edge *replacementEdge = findEdge( replacement, e->e[i]->getOtherNode(originalNode) );

			// create a new edge if it does not exist
			if( !replacementEdge )
				replacementEdge = createEdge( replacement, e->e[i]->getOtherNode(originalNode) );

			// unregister element from the current edge
			e->e[i]->unregisterElement(e);

			// remove the edge if it is desolated
			if( e->e[i]->isDesolate() )
				removeEdge( e->e[i] );

			// replace edge
			e->e[i] = replacementEdge;

			// register element with the new edge
			e->e[i]->registerElement(e);
		}
	}
}

//
// returns the rate at which the stress relaxes in one relaxation-pass
//
float SurfaceCrackGenerator::getRelaxationRate()
{
	return relaxationRate;
}

//
// sets the rate at which the stress relaxes in one relaxation-pass
//
void SurfaceCrackGenerator::setRelaxationRate( float relaxationRate )
{
	this->relaxationRate = relaxationRate;
}

//
// returns the number of iterations for the relaxationstep
//
size_t SurfaceCrackGenerator::getRelaxIterations()
{
	return relaxIterations;
}

//
// sets the number of iterations for the relaxationstep
//
void SurfaceCrackGenerator::setRelaxIterations( size_t relaxIterations )
{
	this->relaxIterations = relaxIterations;
}

//
// returns the strength of crack-resistance
//
float SurfaceCrackGenerator::getMaterialToughness()
{
	return materialToughness;
}

//
// sets the strength of crack-resistance
//
void SurfaceCrackGenerator::setMaterialToughness( float materialToughness )
{
	this->materialToughness = materialToughness;
}

//
// returns the crackpropagation parameter
//
float SurfaceCrackGenerator::getCrackPropagation()
{
	return crackPropagation;
}

//
// sets the crackpropagation parameter
//
void SurfaceCrackGenerator::setCrackPropagation( float crackPropagation )
{
	this->crackPropagation = crackPropagation;
}


//
// returns the number of cracks created per iteration
//
size_t SurfaceCrackGenerator::getCracksPerIteration()
{
	return cracksPerIteration;
}

//
// sets the number of cracks created per iteration
//
void SurfaceCrackGenerator::setCracksPerIteration( size_t cracksPerIteration )
{
	this->cracksPerIteration = cracksPerIteration;
}


//
// SurfaceCrackGenerator::Element -------------------------------------------------------------------------
//

//
// constructor
//
SurfaceCrackGenerator::Element::Element()
{
}

//
// computes normal, area, etc...
//
void SurfaceCrackGenerator::Element::computeProperties()
{
	// compute element center
	for( size_t i=0; i<3; ++i )
		center += n[i]->position;
	center /= 3.0f;

	math::Vec3f u_unormalized = n[1]->position - n[0]->position;
	math::Vec3f         n3vec = n[2]->position - n[0]->position;

	// compute element normal
	u = math::normalize( u_unormalized );
	normal = math::normalize( math::crossProduct( u, n3vec ) );

	v = math::normalize( math::crossProduct( normal, u ) );

	// compute area of the element
	float la = (n1->position - n0->position).getLength(); // compute lengths of the triangle sides
	float lb = (n2->position - n1->position).getLength();
	float lc = (n2->position - n0->position).getLength();
	float s = 0.5f*( la+lb+lc ); // compute the semiperimeter
	area = sqrt( s*(s-la)*(s-lb)*(s-lc) ); // compute the area

	// compute beta (barycentric matrix of the local coordinate frame)
	// the base vectors of beta are the positions of the nodes within the local coordinate frame

	// the first column is the position of the first node within the local coordinate frame which is the orign -> zero
	beta._11 = 0.0f;
	beta._12 = 0.0f;
	beta._13 = 1.0f;

	// the second column is the position of the second node within the local coordinate frame which is the
	// u-axis times the length of the original unormalized u vector
	beta._21 = u_unormalized.getLength();
	beta._22 = 0.0f;
	beta._23 = 1.0f;

	// the third column is the position of the third node within the local coordinate frame which is the
	// vector (e->n[2]->position - e->n[0]->position) projected onto u and v
	beta._31 = math::dotProduct( n3vec, u );
	beta._32 = math::dotProduct( n3vec, v );
	beta._33 = 1.0f;

	// invert beta
	// we dont get away with simply transposing it since betas basis is not orthonormal
	beta.invert();

	// we transpose beta since in the paper the matrix beta has its basevectors in the
	// columns. So we transpose so that we can use the same row/column matrix indicees
	// as they are in the paper
	beta.transpose();
}

//
// transforms the given coordinate in local 2d space into the global 3d space in which the triangle/element lies
//
//
math::Vec3f SurfaceCrackGenerator::Element::convertFromLocalToGlobalSpace( const math::Vec2f &localSpace, bool isVector )
{
	// if the result has to be returned relative to the element origin...
	if( isVector )
		// ...then we dont add the center
		return localSpace.x*u + localSpace.y*v;
	else
		// the result has to be in absolute coordinates
		return n0->position + localSpace.x*u + localSpace.y*v;
}

//
// inverse operation of the above
//
// The argument isVector indicates whether the globalSpace argument is given relative to the element-center (isVector==true)
// or is given in absolut coordinates
//
math::Vec2f SurfaceCrackGenerator::Element::convertFromGlobalToLocalSpace( const math::Vec3f &globalSpace, bool isVector )
{
	// substract the center
	math::Vec3f temp = globalSpace;

	// if the given position is given in absolute world coordinates
	if( !isVector )
		temp -= n0->position;

	// now project the vector onto u and v
	return math::Vec2f( math::dotProduct( temp, u ), math::dotProduct( temp, v ) );
}

//
// This method will take the stress of the parent element and applies it to this element.
// Since these stress quantities are expressed in local 2d coordiate systems a rotation of the
// stress tensor is involved to transform it between the 2 different coordinate systems.
//
void SurfaceCrackGenerator::Element::applySigmaFrom( Element *parent )
{
	// we assume that the parent-element lies in the same plane as the child element
	// thats why we only need to apply a 2d rotation to the stress tensor to transfer
	// it from parent coordinate system in to the child coordinate system

	// create rotation matrix from angle
	// koordinate transformation with the direction cosine
	math::Matrix22f rot = math::Matrix22f( math::dotProduct( parent->u, u ), math::dotProduct( parent->u, v ), math::dotProduct( parent->v, u ), math::dotProduct( parent->v, v ) );
	math::Matrix22f rott = math::transpose( rot );


	sigma = rott * parent->sigma * rot;

	// compute eigenvalues and eigenvectors of sigma
	this->doEigenDecompositionOfSigma();
}

//
// this will compute the eigenvalues and eigenvectors of sigma
// and store the result in the local vectors
//
void SurfaceCrackGenerator::Element::doEigenDecompositionOfSigma()
{
	// compute eigenvectors and eigenvalues of current elements stress tensors
	sigma_eigenValues.clear();
	sigma_eigenVectors.clear();

	std::vector< std::vector<float> > eigenVectors;
	
	doEigenDecomposition( 2, 2, sigma.ma, sigma_eigenValues, eigenVectors );

	for( size_t i=0; i<eigenVectors.size(); ++i )
		sigma_eigenVectors.push_back( math::Vec2f( eigenVectors[i][0], eigenVectors[i][1] ) );
}


//
// this will make sure that the singular value
// of beta will be kept to a certain maximum
//
void SurfaceCrackGenerator::Element::capSingularValueOfBeta( float svThreshold )
{
	beta.transpose();

	// do singular value decomposition of the beta-matrix
	MatDoub mdbeta( 3, 3 );
	for( int i=0;i<3; ++i )
		for( int j=0;j<3; ++j )
			mdbeta[i][j]=beta.m[i][j];

	SVD betasvd( mdbeta );
	//printf( "singular values of beta: %f   %f   %f   (th=%f)\n", betasvd.w[0], betasvd.w[1], betasvd.w[2], svThreshold );

	// cap the singular values to the maximum threshold for beta-singular values
	// cap singular values
	bool recompose = false;
	for( int i=0; i<3; ++i )
		if( betasvd.w[i] > svThreshold )
		{
			betasvd.w[i] = svThreshold;
			recompose = true;
		}

	// if one of the singular values of beta had to be capped, then
	// we have to recompose beta from the capped singular values
	if( recompose )
	{
		// recompose beta
		math::Matrix33f matU, matV, matS;

		matS = math::Matrix33f::Zero();
		matS.m[0][0] = (float)betasvd.w[0];
		matS.m[1][1] = (float)betasvd.w[1];
		matS.m[2][2] = (float)betasvd.w[2];

		matU = math::Matrix33f::Zero();
		for( int i=0;i<3; ++i )
			for( int j=0;j<3; ++j )
				matU.m[i][j]=(float)betasvd.u[i][j];

		matV = math::Matrix33f::Zero();
		for( int i=0;i<3; ++i )
			for( int j=0;j<3; ++j )
				matV.m[i][j]=(float)betasvd.v[i][j];

		beta = matU*matS*math::transpose(matV);
	}

	beta.transpose();
}

//
// computes and adds f_plus and f_minus to the list of
// tensile and compressive forces of the node specified
// with the local index within the element
void SurfaceCrackGenerator::Element::applyTensileAndCompressiveForcesToLocalNode( size_t i )
{
	Element *e = this;

	math::Vec3f f_plus( 0.0f, 0.0f, 0.0f );  // tensile forces
	math::Vec3f f_minus( 0.0f, 0.0f, 0.0f ); // compressive forces

	// compute share of tensile force acting on the current node from the current element (after the equation 3.1 in the phd thesis of iben)
	for( size_t j=0; j<3; ++j )
	{
		float sum_plus = 0.0f;
		float sum_minus = 0.0f;

		for( size_t k=0; k<2; ++k )
			for( size_t l=0; l<2; ++l )
			{
				sum_plus += e->beta.m[j][l]*e->beta.m[i][k]*e->sigma_plus.m[k][l]; // correct version
				sum_minus += e->beta.m[j][l]*e->beta.m[i][k]*e->sigma_minus.m[k][l]; // correct version
			}

		f_plus += e->n[j]->position*sum_plus;
		f_minus += e->n[j]->position*sum_minus;
	}

	f_plus *= -e->area;
	f_minus *= -e->area;

	// add seperated forces to the nodes
	e->n[i]->tensileForces.push_back( f_plus );
	e->n[i]->compressiveForces.push_back( f_minus );
}

//
// computes the change of the strain tensor of the current element
// in respect to the vector component r of the node-th node of the element
//
math::Matrix33f SurfaceCrackGenerator::Element::computeStrainDerivative( size_t node, size_t r )
{
	math::Matrix33f result = math::Matrix33f::Zero();

	for( int j=0; j<3; ++j )
		for( int i=0; i<3; ++i )
		{
			float sum1 = 0.0f;
			float sum2 = 0.0f;
			for( int m=0; m<3; ++m )
			{
				sum1 += n[m]->position[r]*beta.m[node][i]*beta.m[m][j]; // correct
				sum2 += n[m]->position[r]*beta.m[m][i]*beta.m[node][j]; // correct
			}
			result.m[i][j] = 0.5f*( sum1 + sum2 );
		}

	return result;
}


//
// returns the third node which neither equals n1 nor n2
//
SurfaceCrackGenerator::Node *SurfaceCrackGenerator::Element::getOtherNode( Node *n1, Node *n2 )
{
	for( size_t i=0; i<3; ++i )
		if( (n[i] != n1) && (n[i] != n2) )
			return n[i];

	printf( "error in SurfaceCrackGenerator::Element::getOtherNode() : could not find third node from given n1 and n2\n" );
	return 0;
}

//
// returns the third node which neither equals edge->n1 nor edge->n2
//
SurfaceCrackGenerator::Node *SurfaceCrackGenerator::Element::getOtherNode( Edge *edge )
{
	for( size_t i=0; i<3; ++i )
		if( (n[i] != edge->n1) && (n[i] != edge->n2) )
			return n[i];

	printf( "error in SurfaceCrackGenerator::Element::getOtherNode() : could not find third node from given n1 and n2\n" );
	return 0;
}

//
// returns true if one of the element nodes equals the given node n
//
bool SurfaceCrackGenerator::Element::contains( Node *node )
{
	if( (node == n0)||(node == n1)||(node == n2) )
		return true;
	return false;
}

//
// returns the local index of the node within the element
//
size_t SurfaceCrackGenerator::Element::getNodesLocalIndex( Node *node )
{
	for( size_t i=0; i<3; ++i )
		if( n[i] == node )
			return i;
	// node does not belong to the given element
	printf( "error in getNodesLocalIndex() : node does not belong to element\n" );
	return 3;
}

//
// SurfaceCrackGenerator::Node -------------------------------------------------------------------------
//

//
// Constructor
//
SurfaceCrackGenerator::Node::Node()
{
	virtualDisplacement = math::Vec3f( 0.0f, 0.0f, 0.0f );

	compressiveForces.clear();
	tensileForces.clear();
	unbalancedCompressiveForce = math::Vec3f( 0.0f, 0.0f, 0.0f );
	unbalancedTensileForce = math::Vec3f( 0.0f, 0.0f, 0.0f );

	seperationTensor = math::Matrix33f::Zero();

	globalStressTensor = math::Matrix33f::Zero();
}

//
// returns true if no element references this node
//
bool SurfaceCrackGenerator::Node::isDesolate()
{
	return elementRing.empty();
}

//
// adds the given element to the elementring list
//
void SurfaceCrackGenerator::Node::registerElement( Element *e )
{
	elementRing.push_back( e );
}

//
// remvoes the given element to the elementring list
//
void SurfaceCrackGenerator::Node::unRegisterElement( Element *e )
{
	for( std::vector<Element*>::iterator eit=elementRing.begin(); eit != elementRing.end(); ++eit )
		if( *eit == e )
		{
			elementRing.erase( eit );
			return;
		}


	printf( "Error Element was not member of the element ring of given node.\n" );
}

//
// Returns 1 if the node lies on the left side of the given plane (nodeplanedistance < 0.0f)
// or 2 otherwise.
// Returns 0 if point lies on the plane.
//
int SurfaceCrackGenerator::Node::getPlaneSide( const math::Vec3f &normal, const float &distance )
{
	 float planeDistance = math::distancePointPlane( position, normal, distance );

	 // if the planeDistance is 0.0f then the point lies directly on the plane and can not be
	 // used to decide on which side the element lies
	 if( planeDistance == 0.0f )
		 return 0;

	 // signed plane distance
	 if( planeDistance < 0.0f )
		 // point lies on the left side
		 return 1;
	 else
		 // point lies on the right side
		 return 2;
}

//
// this method will do a eigenvalue decomposition of the
// seperationtensor and stores the maximum eigenvalue and its vector
//
void SurfaceCrackGenerator::Node::doEigenDecompositionOfSeperationTensor()
{
	// do eigendecomposition of seperation tensor
	std::vector< float > eigenValues;
	std::vector< std::vector<float> > f_eigenVectors;
	std::vector<math::Vec3f> eigenVectors;
	
	doEigenDecomposition( 3, 3, seperationTensor.ma, eigenValues, f_eigenVectors );

	for( size_t i=0; i<f_eigenVectors.size(); ++i )
		eigenVectors.push_back( math::Vec3f( f_eigenVectors[i][0], f_eigenVectors[i][1], f_eigenVectors[i][2] ) );

/*
	// find the greatest absolute value of the eigenvalue
	maxEigenvalue = fabs(eigenValues[0]);
	maxEigenvaluesEigenvector = eigenVectors[0];

	for( size_t i=1; i<eigenValues.size(); ++i )
	{
		if( fabs(eigenValues[i]) > maxEigenvalue )
		{
			maxEigenvalue = fabs(eigenValues[i]);
			maxEigenvaluesEigenvector = eigenVectors[i];
		}
	}
*/

	// FLAG!
	// stress field...
	for( size_t i=0; i<eigenValues.size(); ++i )
		eigenValues[i] = -eigenValues[i];
	

	// find the greatest absolute value of the eigenvalue
	maxEigenvalue = eigenValues[0];
	maxEigenvaluesEigenvector = eigenVectors[0];

	for( size_t i=1; i<eigenValues.size(); ++i )
	{
		if( eigenValues[i] > maxEigenvalue )
		{
			maxEigenvalue = eigenValues[i];
			maxEigenvaluesEigenvector = eigenVectors[i];
		}
	}

}


//
// SurfaceCrackGenerator::Edge -------------------------------------------------------------------------
//

//
// constructor
//
SurfaceCrackGenerator::Edge::Edge( Node *_n1, Node *_n2 ):left(0), right(0), n1(_n1), n2(_n2)
{
};

//
// returns true if no elements references this edge
//
bool SurfaceCrackGenerator::Edge::isDesolate()
{
	if( left || right )
		return false;
	return true;
}

//
// convinience function to get the other node of the two nodes which are referenced by the edge
//
SurfaceCrackGenerator::Node *SurfaceCrackGenerator::Edge::getOtherNode( Node *n )
{
	if( n == n1 )
		return n2;
	else
	if( n == n2 )
		return n1;
	else
		printf( "error in SurfaceCrackGenerator::Edge::getOtherNode() : given node n does not belong to the edge\n" );
	return 0;
}

//
// returns true if the edge contains the given node
//
bool SurfaceCrackGenerator::Edge::contains( Node *n )
{
	if( (n1 == n)||(n2 == n) )
		return true;
	return false;
}

//
// returns true if the edge has only one element reference instead of two
//
bool SurfaceCrackGenerator::Edge::isBoundaryEdge()
{
	// if both element references are valid...
	if( left && right )
		// then this is not a boundary edge
		return false;
	// boundary or desolate edge
	return true;
}

//
// convinience function to compute the intersection of the edge with a given plane
//
bool SurfaceCrackGenerator::Edge::intersectsPlane( const math::Vec3f &normal, const float &distance, math::Vec3f &intersectionPoint )
{
	return math::intersectionRayPlane( math::Ray3d( n1->position, n2->position ), normal, distance, intersectionPoint );
}

//
// assigns the given element to the left or right wing of the edge (dependand on which wing is 0)
//
void SurfaceCrackGenerator::Edge::registerElement( Element *e )
{
	if( !left )
	{
		left = e;
	}else
	if( !right )
	{
		right = e;
	}else
	{
		printf( "error - edge must not belong to more than 2 elements\n" );
	}
}

//
// sets either the left or right reference to zero when it references the given element
//
void SurfaceCrackGenerator::Edge::unregisterElement( Element *e )
{
	if( left == e )
	{
		left = 0;
	}else
	if( right == e )
	{
		right = 0;
	}else
	{
		printf( "error - edge did not reference the given element\n" );
	}
}