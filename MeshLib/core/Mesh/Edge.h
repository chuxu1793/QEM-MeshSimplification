/*!
*      \file Edge.h
*      \brief Base class of edge
*	   \author David Gu
*      \date 10/07/2010
*
*/


#ifndef _MESHLIB_EDGE_H_
#define _MESHLIB_EDGE_H_

#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <string>

namespace MeshLib{

class CHalfEdge;
class CVertex;

/*!
\brief CEdge class, which is the base class of all kinds of edge classes
*/
class CEdge
{
public:
	/*!
		CEdge constructor, set both halfedge pointers to be NULL.
	*/
	CEdge(){ m_halfedge[0] = NULL; m_halfedge[1] = NULL; };
	/*!
		CEdge destructor.
	*/
	~CEdge(){};
	/*!
		Edge ID
	 */
	int & id() { return m_id; };

	/*!
		The halfedge attached to the current edge
		\param id either 0 or 1
		\return the halfedge[id]
	*/
	CHalfEdge * & halfedge( int id ) { assert( 0<=id && id < 2 ); return m_halfedge[id];}; // 根据id索引对应的半边

	/*!	
		whether the edge is on the boundary.  判断边是否是边界
	*/
	bool boundary()
	{
		return (m_halfedge[0] == NULL && m_halfedge[1] != NULL) || (m_halfedge[0] != NULL && m_halfedge[1] == NULL); // 一个半边为空，另一个不为空则说明当前边是边界
	};

	/*!
		The dual halfedge to the input halfedge
		\param he halfedge attached to the current edge
		\return the other halfedge attached to the current edge
	*/
	CHalfEdge *&other(CHalfEdge *he) // 根据当前边的其中一个半边获取另一个半边
	{
		return (he != m_halfedge[0]) ? m_halfedge[0] : m_halfedge[1];
	};

	/*!
		The string of the current edge.
	*/
	std::string & string() { return m_string; };
	/*!
		Read the traits from the string.
	*/
	void _from_string() {};
	/*!
		Save the traits to the string.
	*/
	void _to_string() {};
	double getk() { return k; }
	void setk(double k1) { k = k1; }
protected:
	/*!
		Pointers to the two halfedges attached to the current edge. 指向附加到当前边的两个半边
	*/
	CHalfEdge      * m_halfedge[2];
	/*!
		The string associated to the current edge.
	*/
    std::string      m_string; // 边所关联的字符串
	/*!
		Edge ID
	 */
	int				 m_id; // 边的ID
	double k;
};





}//name space MeshLib

#endif //_MESHLIB_EDGE_H_ defined