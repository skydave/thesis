/*---------------------------------------------------------------------

MAC bases fluid simulator which uses a staggered grid and particles
or tracking the fluid. In addition to basic MAC simulation the PIC
and FLIP methods can be used to achieve better animations.

----------------------------------------------------------------------*/
#pragma once
#include <vector>
#include <hash_map>
#include <math.h>

#include <math/Math.h>


///
/// \brief marker particle used by the fluidsimulator 
///
struct Particle2d
{
	Particle2d( math::Vec2f _position ) : position(_position), velocity()
	{
	}
	Particle2d( )
	{
	}
	math::Vec2f position;
	math::Vec2f velocity;
};


///
/// \brief  Full featured MAC simulator with PIC/FLIP extension
///
class FluidSimulator2d
{
//protected:
public:

	/// \brief fluid cell which is used to solve the equations
	/// The cell structure is used to discretize the simulation domain
	/// the cell are staggered, that means the velocity components are
	/// not stored in the center, but on the faces of the cell
	///
	struct Cell
	{
		/// \brief locates the cells within the domain
		/// Each cell represents a portion of the simulation domain.
		/// The coordinates are used to locate the cells within the domain
		///
		struct Coordinate
		{
			int                                                              i,j; // discrete cell coordinates

			Coordinate();                                                         // standard constructor
			Coordinate( int _i, int _j );                                         // constructs the cell from given coordinates

			Coordinate        getNeighbourCoordinate( int neighbourIndex ) const; // returns the Coordinate within the grid of the index-specified neighbour
			Coordinate getInverseNeighbourCoordinate( int neighbourIndex ) const; // this method returns the inverse coordinate of given neighbourindex
		};

		/// Each cell can be one of the following types:
		enum Type
		{
			Fluid, 		// Fluidcells are cells which will go into the navier-stokes solving process
			Solid,      // Solid cells are those cells which lie within a solid object and are used to keep the boundary conditions
			Air         // air cells are cells which sourround the fluid surface and are kind of a buffer region which is neccessary for particle tracing etc.
		};


		Cell();                                                                   ///< constructor

		void                 setNeighbour( unsigned int index, Cell *neighbour ); ///< Registers reference to the index-specified neighbour
		void                               removeNeighbour( unsigned int index ); ///< Unregisters reference to the index-specified neighbour
		Cell                                     *getNeighbour( int ri, int rj ); ///< retrieve the neighbour from given relative coordinates. ri, rj each are -1, 0 or +1

		bool                                                       isFluidCell(); ///< returns true if the cell is of type fluid

		int                                                                layer; ///< some sort of importance filter: -1 means unused/unimportant; 0 means "most important"; 1+ decreased importance
		Type                                                                type; ///< type of the fluid cell
		math::Vec2f                                                     velocity; ///< velocity components (components are assumed to lie on cell-faces)
		math::Vec2f                                               velocityChange; ///< for the FLIP method we need the change in velocity
		math::Vec2f                                                  oldVelocity; ///< to compute the change in velocity we need the velocity of the previous step

		bool                                                    xNeighboursFluid; ///< this bool tells wether the x component of the velocity lies next to a fluid cell
		bool                                                    yNeighboursFluid; ///< this bool tells wether the y component of the velocity lies next to a fluid cell

		Cell                                                      *neighbours[8]; ///< references to all 10 neighbouring cells which are used for interpolation (interpolation-neighbours) and simulation
		bool                                     allInterpolationNeighboursExist; ///< this routine tells whether all 5 neighbouring cells for interpolation have been created and are present within the hashtable


		math::Vec2f                                                 tempVelocity; ///< temp variables for the velocity components (used by the solver)
		float                                                           pressure; ///< temp variable which is used by the solver and will hold the pressure-correction term for this cell
		float                                                     signedDistance; ///< signed distance to the closest fluid surface point for each component
		float                                                                sum;

		// extensions...
		bool                                                               rigid; ///< used by the sand simulator to identify rigid cells
		math::Vec2f                                                       center; ///< we need the position of the cell centers for the rigidification
		math::Matrix22f                                             strainTensor; ///< this tensor represents the strain situatiopn for the current cell

		// viscoelastic fluid extensions...
		math::Matrix22f                                                     temp; //

		float                                                         meanStress;
		float                                                        shearStress;
		float                                           vonMisesEquivalentStress;

		int                                                               layer2;
		math::Matrix22f                                             stressTensor; ///< derived from strain
	};

	///
	/// \brief StaticSolid is a abstract interface which is used do define non-moving solid cells within the simulationdomain
	///
	struct StaticSolid
	{
		virtual bool occupies( const math::Vec2f &cellCenter )=0; ///< this function is used to indirectly voxelize the obstacle
		virtual void               computeTransformationMatrix();
		virtual math::Matrix44f        getTransformationMatrix();
		virtual math::Matrix44f getInverseTransformationMatrix();
		void                  setTranslation( float x, float y );
		void                         setRotation( float zAngle );

	protected:
		math::Vec2f                                  translation;
		float                                          zRotation;
		math::Matrix44f                                transform;
		math::Matrix44f                         inverseTransform;
	};

	/// \brief used by the cell-hash-map
	/// This class is used to used by the stdext::hash_map container to sort the key values and to get
	/// hash values for keys
	///
	struct hash_compare : public stdext::hash_compare<FluidSimulator2d::Cell::Coordinate>
	{
		size_t operator() (const FluidSimulator2d::Cell::Coordinate& key) const;  ///< hash function which returns an hashvalue for the given key
		bool operator() (const FluidSimulator2d::Cell::Coordinate& key1, const FluidSimulator2d::Cell::Coordinate& key2) const; ///< comperator for internal sorting (dont ask me why hash_table is sorting at all)
	};



public:
	FluidSimulator2d();                                                                                ///< constructor
	~FluidSimulator2d();                                                                               ///< destructor

	std::vector<Particle2d>                                                           markerParticles; ///< particles used to track the volume of the fluid in space

//protected:
public:
	float                                                                                    cellSize; ///< width/height and depth of a cell - cells are always cubic
	float                                                                                dtMin, dtMax; ///< min max range of dt for one simulationstep
	bool                                                                       particleBasedAdvection; ///< this bool tells, wether the advection has to be done through particle velocities(true) or a semi-lagrange-method(false)
	float                                                                               PICFLIPWeight; ///< as proposed in the bridson paper, the result of the PIC and the FLIP method are blended together with a specified weight 1.0f means 100% FLIP and 0% PIC

	math::BoundingBox                                                                simulationDomain; ///< overall spatial restriction of the simulation
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>                                  gridCells; ///< a hashtable which holds all cells currently in use - this is the staggered grid

	float                                                                         atmosphericPressure; ///< this constant defines the pressure within the atmosphere (non-fluid and non-solid domain)
	float                                                                                   viscosity; ///< internal friction within the fluid

	std::vector<StaticSolid *>                                                           staticSolids; ///< list of non-moving objects


public:
	void                                                           advanceFrame( float timePerFrame ); ///< advances a specified timestep with a dynamic amount of substeps
	void                                                               advanceStep( float deltaTime ); ///< advances the fluid the given timedelta

	void                                                                                reset( void ); ///< clears all gridcells and all marker particles

	float                                                                        CFLDeltaTime( void ); ///< computes a timeDelta so that the condition velocity*timeDelta < cellSize is always true

	void                                                               setCellSize( float _cellSize ); ///< setup the size of one grid cell in each dimension
	float                                                                               getCellSize(); ///< returns cellsize
	void                                                setDeltaTimeRange( float dtmin, float dtmax ); ///< set the range in which the delta time must be for one simulation step
	void                        setSimulationDomain( const math::Vec2f &min, const math::Vec2f &max ); ///< sets the spatial boundaries of the simulation
	void                                                             setViscosity( float _viscosity ); ///< sets the simple-fluid visosity
	Cell                                                               *getCell( size_t i, size_t j ); ///< returns a pointer to the cell which lies at the specified coordinate - if the cell does not exist, null is returned

	void                                                   addStaticSolid( StaticSolid *staticSolid ); ///< adds a non moving obstacle to the fluid simulation domain
	size_t                                                                      getStaticSolidCount(); ///< returns the number of static solids
	StaticSolid                                                       *getStaticSolid( size_t index ); ///< returns the index specified solid

	virtual void                        onCellCreated( const Cell::Coordinate *coord, Cell *newCell ); ///< this method will be called when a new cell just has been created - override it to get a hand on newly created cells

	void                                                           transferParticleVelocitiesToGrid(); ///< this method is used by updategrid when pic/flip us used and may be used to define the grid velocities in an intuitive manner

//protected:
	virtual void                                                                   updateGrid( void ); ///< updates the grid (remarks all cells, prepares the grid for velocity update)

	virtual void                                                           updateVelocity( float dt ); ///< This method performs the actual solving of the navier-stokes equation

	virtual void                                                            moveParticles( float dt ); ///< This method will move the marker particles depending on the grid-velocity through the grid

	virtual void                                                        setBoundaryConditions( void ); ///< this method will set the velocities on solid cells which border non-solid cells so that free-slip, no-slip, or frictional boundary conditions are met
	virtual void                                                           solveViscosity( float dt ); ///< this method will solve the viscosity term...
	virtual void                                           applyGravity( float dt, float x, float y ); ///< will add dt*math::Vec3f(x,y) to the respective velocity components, which neighbour fluid cells
	virtual void                                                            solvePressure( float dt ); ///< this will solve the pressure-term and will make the grid velocities divergence free
	virtual void                                                     advectGridVelocities( float dt ); ///< if simple MAC simulation is used, the velocities of the grid have to be advected...


	void sweepUpdateDistances( Cell *cell, int horizontalNeighbourIndex, int verticalNeighbourIndex ); ///< This is the actual distance computation proposed in [Zhao2005]
	void                                                               computeDistanceToFluid( void ); ///< Computes the signed distance to the fluid surface for each cell in the grid using fast sweeping [Zhao 2005].
	void                                sweepVelocityUV( int iStart, int iEnd, int jStart, int jEnd ); ///< General implementation of the sweeping for velocity extrapolation for the u- and v-component of the velocity
	void                                                                extrapolateVelocities( void ); ///< this method will compute all the velocity components of grid cell-components which dont border fluid cells

	float              getInterpolatedVelocityValue( float x, float y, unsigned int componentIndex  ); ///< returns the index-specified trilinear interpolated velocity-component from the grid.
	float                getInterpolatedChangeValue( float x, float y, unsigned int componentIndex  ); ///< returns the index-specified trilinear interpolated velocity-component change from the grid.
	math::Vec2f                                                       getVelocity( float x, float y ); ///< This function does look up the velocity within the simulationDomain
	math::Vec2f                                                 getVelocityChange( float x, float y ); ///< This function does look up the change in velocity within the simulationDomain (used for FLIP)

	math::Vec2f                         traceParticle( const math::Vec2f &position, float deltaTime ); ///< This method returns the position of the given point at t - deltaTime (used for semi-lagrange style advection)


	void                                    updateNeighbours( Cell *cell, const Cell::Coordinate &c ); ///< used to update neighbouring cells when a cell is added
	void                                                updateNeighbours( const Cell::Coordinate &c ); ///< used to update neighbouring cells when a cell is removed
};