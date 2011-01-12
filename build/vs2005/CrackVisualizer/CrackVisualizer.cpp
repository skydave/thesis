/*---------------------------------------------------------------------

The CrackVisualizer is a 3dsmax texture plugin which is used to render
the Cracks on a mesh.

----------------------------------------------------------------------*/
#include "CrackVisualizer.h"
#include <map>
#include <vector>
#define CrackVisualizer_CLASS_ID	Class_ID(0x8da1e269, 0x66bb7bd6)

#define NSUBTEX		1 // TODO: number of sub-textures supported by this plugin 
#define COORD_REF	0

#define PBLOCK_REF	1

class CrackVisualizer;

class CrackVisualizerSampler: public MapSampler {
	CrackVisualizer	*tex;
	public:
		CrackVisualizerSampler() { tex= NULL; }
		CrackVisualizerSampler(CrackVisualizer *c) { tex= c; }
		void Set(CrackVisualizer *c) { tex = c; }
		AColor Sample(ShadeContext& sc, float u,float v);
		AColor SampleFilter(ShadeContext& sc, float u,float v, float du, float dv);
		float SampleMono(ShadeContext& sc, float u,float v);
		float SampleMonoFilter(ShadeContext& sc, float u,float v, float du, float dv);
	} ;


class CrackVisualizer : public Texmap {
	public:

		// Parameter block
		IParamBlock2	*pblock;	//ref 0

		Texmap			*subtex[NSUBTEX]; //array of sub-materials
		static ParamDlg *uvGenDlg;
		UVGen			*uvGen;
		Interval		ivalid;

		//From MtlBase
		ParamDlg* CreateParamDlg(HWND hwMtlEdit, IMtlParams *imp);
		BOOL SetDlgThing(ParamDlg* dlg);
		void Update(TimeValue t, Interval& valid);
		void Reset();
		Interval Validity(TimeValue t);
		ULONG LocalRequirements(int subMtlNum);

		//TODO: Return the number of sub-textures
		int NumSubTexmaps() { return NSUBTEX; }
		//TODO: Return the pointer to the 'i-th' sub-texmap
		Texmap* GetSubTexmap(int i) { return subtex[i]; }
		void SetSubTexmap(int i, Texmap *m);
		TSTR GetSubTexmapSlotName(int i);
		
		//From Texmap
		RGBA EvalColor(ShadeContext& sc);
		float EvalMono(ShadeContext& sc);
		Point3 EvalNormalPerturb(ShadeContext& sc);

		//TODO: Returns TRUE if this texture can be used in the interactive renderer
		BOOL SupportTexDisplay() { return FALSE; }
		void ActivateTexDisplay(BOOL onoff);
		DWORD_PTR GetActiveTexHandle(TimeValue t, TexHandleMaker& thmaker);
		//TODO: Return UV transformation matrix for use in the viewports
		void GetUVTransform(Matrix3 &uvtrans) { uvGen->GetUVTransform(uvtrans); }
		//TODO: Return the tiling state of the texture for use in the viewports
		int GetTextureTiling() { return  uvGen->GetTextureTiling(); }
		int GetUVWSource() { return uvGen->GetUVWSource(); }
		UVGen *GetTheUVGen() { return uvGen; }
		
		//TODO: Return anim index to reference index
		int SubNumToRefNum(int subNum) { return subNum; }
		
		// Loading/Saving
		IOResult Load(ILoad *iload);
		IOResult Save(ISave *isave);

		//From Animatable
		Class_ID ClassID() {return CrackVisualizer_CLASS_ID;}		
		SClass_ID SuperClassID() { return TEXMAP_CLASS_ID; }
		void GetClassName(TSTR& s) {s = GetString(IDS_CLASS_NAME);}

		RefTargetHandle Clone( RemapDir &remap );
		RefResult NotifyRefChanged(Interval changeInt, RefTargetHandle hTarget, 
			PartID& partID,  RefMessage message);


		int NumSubs() { return 2+NSUBTEX; }
		Animatable* SubAnim(int i); 
		TSTR SubAnimName(int i);

		// TODO: Maintain the number or references here 
		int NumRefs() { return 2+NSUBTEX; }
		RefTargetHandle GetReference(int i);
		void SetReference(int i, RefTargetHandle rtarg);

		int	NumParamBlocks() { return 1; }					// return number of ParamBlocks in this instance
		IParamBlock2* GetParamBlock(int i) { return pblock; } // return i'th ParamBlock
		IParamBlock2* GetParamBlockByID(BlockID id) { return (pblock->ID() == id) ? pblock : NULL; } // return id'd ParamBlock

		void DeleteThis() { delete this; }		
		//Constructor/Destructor

		CrackVisualizer();
		~CrackVisualizer();

		// dave: added
		std::map<int, std::map<DWORD, std::vector<std::pair<DWORD, DWORD> > > > adjBoundaryEdges;  // maps nodeID to a map which maps a list of boundary edges to each vertex
		void findAdjBoundaryEdges( int nodeID, RenderInstance* inst );
		int BuildMaps(TimeValue t, RenderMapsContext &rmc);
};



class CrackVisualizerClassDesc : public ClassDesc2 {
	public:
	int 			IsPublic() { return TRUE; }
	void *			Create(BOOL loading = FALSE) { return new CrackVisualizer(); }
	const TCHAR *	ClassName() { return GetString(IDS_CLASS_NAME); }
	SClass_ID		SuperClassID() { return TEXMAP_CLASS_ID; }
	Class_ID		ClassID() { return CrackVisualizer_CLASS_ID; }
	const TCHAR* 	Category() { return GetString(IDS_CATEGORY); }

	const TCHAR*	InternalName() { return _T("CrackVisualizer"); }	// returns fixed parsable name (scripter-visible name)
	HINSTANCE		HInstance() { return hInstance; }					// returns owning module handle
	

};

static CrackVisualizerClassDesc CrackVisualizerDesc;
ClassDesc2* GetCrackVisualizerDesc() { return &CrackVisualizerDesc; }




enum { crackvisualizer_params };


//TODO: Add enums for various parameters
enum { 
	pb_spin,
	pb_color,
	pb_coords,
};




static ParamBlockDesc2 crackvisualizer_param_blk ( crackvisualizer_params, _T("params"),  0, &CrackVisualizerDesc, 
	P_AUTO_CONSTRUCT + P_AUTO_UI, PBLOCK_REF, 
	//rollout
	IDD_PANEL, IDS_PARAMS, 0, 0, NULL,
	// params
	pb_spin, 			_T("spin"), 		TYPE_FLOAT, 	P_ANIMATABLE, 	IDS_SPIN, 
		p_default, 		0.1f, 
		p_range, 		0.0f,1000.0f, 
		p_ui, 			TYPE_SPINNER,		EDITTYPE_FLOAT, IDC_EDIT,	IDC_SPIN, 0.01f, 
		end,
	pb_color, _T("color"), TYPE_RGBA, P_ANIMATABLE, IDS_COLOR,
		p_default, Color(0.0f, 0.0f, 0.0f),
		p_ui, TYPE_COLORSWATCH, CS_COLOR,
	end,
	pb_coords,			_T("coords"),		TYPE_REFTARG,	P_OWNERS_REF,	IDS_COORDS,
		p_refno,		COORD_REF, 
		end,
	end
	);




ParamDlg* CrackVisualizer::uvGenDlg;

//--- CrackVisualizer -------------------------------------------------------
CrackVisualizer::CrackVisualizer()
{
	for (int i=0; i<NSUBTEX; i++) subtex[i] = NULL;
	//TODO: Add all the initializing stuff
	pblock = NULL;
	CrackVisualizerDesc.MakeAutoParamBlocks(this);
	uvGen = NULL;
	Reset();
}

CrackVisualizer::~CrackVisualizer()
{

}

void CrackVisualizer::findAdjBoundaryEdges( int nodeID, RenderInstance* inst )
{
	adjBoundaryEdges[ nodeID ].clear();

	// build the openEdges bitarray for the current mesh
	BitArray openEdges;
	inst->mesh->FindOpenEdges( openEdges );

	for( size_t k=0; k<inst->mesh->numFaces; ++k )
	{
		Face &f = inst->mesh->faces[k];

		// for each edge of that face
		for( int i=0; i<3; ++i )
		{
			// check if it is a boundary edge
			DWORD edgeIdx = f.GetEdgeIndex( f.getVert(i), f.getVert((i+1)%3) );

			// if the current edge is not a boundary edge
			if( openEdges[k*3+edgeIdx] == 0 )
				continue;

			// it is:
			// store edge 
			adjBoundaryEdges[ nodeID ][f.getVert(i)].push_back( std::make_pair( f.getVert(i), f.getVert((i+1)%3) ) );
			adjBoundaryEdges[ nodeID ][f.getVert((i+1)%3)].push_back( std::make_pair( f.getVert(i), f.getVert((i+1)%3) ) );
		}
	}
}

//
//
//
int CrackVisualizer::BuildMaps(TimeValue t, RenderMapsContext &rmc)
{
	//MessageBox( 0, "test", "test", MB_OK );
	int nodeID = rmc.NodeRenderID();

	// get the renderinstance for the current node
	RenderInstance* inst = rmc.GetGlobalContext()->GetRenderInstance(nodeID);

	if( (inst==NULL) || (inst->mesh==NULL) || (NULL == inst->mesh->faces)  )
		return Texmap::BuildMaps( t, rmc );


	findAdjBoundaryEdges( nodeID, inst );


	// done
	return Texmap::BuildMaps( t, rmc );
}

//From MtlBase
void CrackVisualizer::Reset() 
{
	if (uvGen) uvGen->Reset();
	else ReplaceReference( 0, GetNewDefaultUVGen());	
	//TODO: Reset texmap back to its default values

	ivalid.SetEmpty();

}

void CrackVisualizer::Update(TimeValue t, Interval& valid) 
{	
	//TODO: Add code to evaluate anything prior to rendering
}

Interval CrackVisualizer::Validity(TimeValue t)
{
	//TODO: Update ivalid here
	return ivalid;
}

ParamDlg* CrackVisualizer::CreateParamDlg(HWND hwMtlEdit, IMtlParams *imp) 
{
	IAutoMParamDlg* masterDlg = CrackVisualizerDesc.CreateParamDlgs(hwMtlEdit, imp, this);
	uvGenDlg = uvGen->CreateParamDlg(hwMtlEdit, imp);
	masterDlg->AddDlg(uvGenDlg);
	//TODO: Set the user dialog proc of the param block, and do other initialization	
	return masterDlg;	
}

BOOL CrackVisualizer::SetDlgThing(ParamDlg* dlg)
{	
	if (dlg == uvGenDlg)
		uvGenDlg->SetThing(uvGen);
	else 
		return FALSE;
	return TRUE;
}

void CrackVisualizer::SetSubTexmap(int i, Texmap *m) 
{
	ReplaceReference(i+2,m);
	//TODO Store the 'i-th' sub-texmap managed by the texture
}

TSTR CrackVisualizer::GetSubTexmapSlotName(int i) 
{	
	//TODO: Return the slot name of the 'i-th' sub-texmap
	return TSTR(_T(""));
}


//From ReferenceMaker
RefTargetHandle CrackVisualizer::GetReference(int i) 
{
	//TODO: Return the references based on the index	
	switch (i) {
		case 0: return uvGen;
		case 1: return pblock;
		default: return subtex[i-2];
		}
}

void CrackVisualizer::SetReference(int i, RefTargetHandle rtarg) 
{
	//TODO: Store the reference handle passed into its 'i-th' reference
	switch(i) {
		case 0: uvGen = (UVGen *)rtarg; break;
		case 1:	pblock = (IParamBlock2 *)rtarg; break;
		default: subtex[i-2] = (Texmap *)rtarg; break;
	}
}

//From ReferenceTarget 
RefTargetHandle CrackVisualizer::Clone(RemapDir &remap) 
{
	CrackVisualizer *mnew = new CrackVisualizer();
	*((MtlBase*)mnew) = *((MtlBase*)this); // copy superclass stuff
	//TODO: Add other cloning stuff
	BaseClone(this, mnew, remap);
	return (RefTargetHandle)mnew;
}

	 
Animatable* CrackVisualizer::SubAnim(int i) 
{
	//TODO: Return 'i-th' sub-anim
	switch (i) {
		case 0: return uvGen;
		case 1: return pblock;
		default: return subtex[i-2];
		}
}

TSTR CrackVisualizer::SubAnimName(int i) 
{
	//TODO: Return the sub-anim names
	switch (i) {
		case 0: return GetString(IDS_COORDS);		
		case 1: return GetString(IDS_PARAMS);
		default: return GetSubTexmapTVName(i-1);
		}
}

RefResult CrackVisualizer::NotifyRefChanged(Interval changeInt, RefTargetHandle hTarget, 
   PartID& partID, RefMessage message ) 
{
	//TODO: Handle the reference changed messages here	
	return(REF_SUCCEED);
}

IOResult CrackVisualizer::Save(ISave *isave) 
{
	//TODO: Add code to allow plugin to save its data
	return IO_OK;
}

IOResult CrackVisualizer::Load(ILoad *iload) 
{ 
	//TODO: Add code to allow plugin to load its data
	return IO_OK;
}

AColor CrackVisualizer::EvalColor(ShadeContext& sc)
{
	if (gbufID) sc.SetGBufferID(gbufID);

	float dist = pblock->GetFloat( pb_spin, sc.CurTime() )*0.1f;
	float distSquared = dist*dist;
	float minDist = 9999999999999999999.0f;

	// we must be sure that minDist is a value greater than dist
	// ...

	//AColor edgeColor = AColor (1.0f,0.0f,0.0f,1.0f);
	AColor edgeColor = pblock->GetAColor( pb_color, sc.CurTime() );
	edgeColor.a = 1.0f;
	


	int nodeID = sc.NodeID();

	if( !sc.globContext )
		return AColor (0.0f,0.0f,0.0f,0.0f);

	RenderInstance* inst = sc.globContext->GetRenderInstance(nodeID);
	if( (inst==NULL) || (inst->mesh==NULL) || NULL == inst->mesh->faces || inst->mesh->getNumFaces() <= sc.FaceNumber() )
	{
		return AColor (0.0f,0.0f,0.0f,0.0f);
	} 

	// if an entry for the current nodeID doesnt exist 
	if( adjBoundaryEdges.find( nodeID ) == adjBoundaryEdges.end() )
		// build the table
		findAdjBoundaryEdges( nodeID, inst );

	int faceIndex = sc.FaceNumber();
	Face& f = inst->mesh->faces[faceIndex];

	// compute Position of p
	Point3 bary = sc.BarycentricCoords();
	Point3 p = bary[0]*inst->mesh->getVert(f.getVert(0)) + bary[1]*inst->mesh->getVert(f.getVert(1)) + bary[2]*inst->mesh->getVert(f.getVert(2));


	// p is not close to any boundary edge
	// check if p close to any vertex which neighbours a boundaryedge from another triangle
	for( int i=0; i<3; ++i )
	{
		// if wireframe
		if(0)
		{
			DWORD edgeIdx = f.GetEdgeIndex( f.getVert(i), f.getVert((i+1)%3) );

			// get vertex positions
			Point3 v0 = inst->mesh->getVert(f.getVert(i));
			Point3 v1 = inst->mesh->getVert(f.getVert(i+1)%3);

			// compute distance p <-> edge v0, v1
			//float edgeDistance = distancePointLine( p, v0, v1 );
			float edgeDistance = Dist3DPtToLine( &p, &v0, &v1 );
			edgeDistance = edgeDistance*edgeDistance;

			// if distance of p is closer then 1/10 of the distance of v2 to that edge
			if( edgeDistance < minDist )
				minDist = edgeDistance;
		}

		// if there is any incident boundary edge to the current vertex, than we know that it is a
		// boundary vertex
		if( !adjBoundaryEdges[nodeID][f.getVert(i)].empty() )
		{
			// current vertex is a boundary vertex
			// comute distance of p to that vertex
			float vertexDistance = (inst->mesh->getVert( f.getVert(i) ) - p).LengthSquared();

			if( vertexDistance < minDist )
				minDist = vertexDistance;

			// check all boundary edges which are adjacent to the vertex and may
			// come from other faces
			for( int j = 0; j<adjBoundaryEdges[nodeID][f.getVert(i)].size(); ++j  )
			{
				// compute distance to that edge
				Point3 v0 = inst->mesh->getVert( adjBoundaryEdges[nodeID][f.getVert(i)][j].first );
				Point3 v1 = inst->mesh->getVert( adjBoundaryEdges[nodeID][f.getVert(i)][j].second );

				// compute dotproduct
				Point3 vec = p - v0;
				Point3 direction = Normalize( v1 - v0 );
				float maxLength = Length( v1 - v0 );

				float dp = DotProd( vec, direction  );

				if( (dp<0.0f)||(dp>maxLength) )
					continue;

				float edgeDistance = LengthSquared( vec - dp*direction );

				if( edgeDistance < minDist )
					minDist = edgeDistance;
			}
		}
	}

	if( minDist < distSquared )
		return edgeColor;


	return AColor (0.0f,0.0f,0.0f,0.0f);}

float CrackVisualizer::EvalMono(ShadeContext& sc)
{
	//TODO: Evaluate the map for a "mono" channel
	return Intens(EvalColor(sc));
}

Point3 CrackVisualizer::EvalNormalPerturb(ShadeContext& sc)
{
	//TODO: Return the perturbation to apply to a normal for bump mapping
	return Point3(0, 0, 0);
}

ULONG CrackVisualizer::LocalRequirements(int subMtlNum)
{
	//TODO: Specify various requirements for the material
	// MTLREQ_PREPRO will cause the buildmaps to be called every frame
	return uvGen->Requirements(subMtlNum) | MTLREQ_PREPRO; 
}

void CrackVisualizer::ActivateTexDisplay(BOOL onoff)
{
	//TODO: Implement this only if SupportTexDisplay() returns TRUE
}

DWORD_PTR CrackVisualizer::GetActiveTexHandle(TimeValue t, TexHandleMaker& thmaker)
{
	//TODO: Return the texture handle to this texture map
	return 0;
}
