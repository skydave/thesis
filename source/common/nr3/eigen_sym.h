/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once

#include "nr3.h"


void eigsrt(VecDoub_IO &d, MatDoub_IO *v=NULL);


/// \brief eigenvalue decomposition helper class (from numerical recipes 3rd)
struct Jacobi
{
	const Int n;
	MatDoub a,v;
	VecDoub d;
	Int nrot;
	const Doub EPS;

	Jacobi(MatDoub_I &aa) : n(aa.nrows()), a(aa), v(n,n), d(n), nrot(0),
		EPS(numeric_limits<Doub>::epsilon())
	{
		Int i,j,ip,iq;
		Doub tresh,theta,tau,t,sm,s,h,g,c;
		VecDoub b(n),z(n);
		for (ip=0;ip<n;ip++) {
			for (iq=0;iq<n;iq++) v[ip][iq]=0.0;
			v[ip][ip]=1.0;
		}
		for (ip=0;ip<n;ip++) {
			b[ip]=d[ip]=a[ip][ip];
			z[ip]=0.0;
		}
		for (i=1;i<=50;i++) {
			sm=0.0;
			for (ip=0;ip<n-1;ip++) {
				for (iq=ip+1;iq<n;iq++)
					sm += abs(a[ip][iq]);
			}
			if (sm == 0.0) {
				eigsrt(d,&v);
				return;
			}
			if (i < 4)
				tresh=0.2*sm/(n*n);
			else
				tresh=0.0;
			for (ip=0;ip<n-1;ip++) {
				for (iq=ip+1;iq<n;iq++) {
					g=100.0*abs(a[ip][iq]);
					if (i > 4 && g <= EPS*abs(d[ip]) && g <= EPS*abs(d[iq]))
							a[ip][iq]=0.0;
					else if (abs(a[ip][iq]) > tresh) {
						h=d[iq]-d[ip];
						if (g <= EPS*abs(h))
							t=(a[ip][iq])/h;
						else {
							theta=0.5*h/(a[ip][iq]);
							t=1.0/(abs(theta)+sqrt(1.0+theta*theta));
							if (theta < 0.0) t = -t;
						}
						c=1.0/sqrt(1+t*t);
						s=t*c;
						tau=s/(1.0+c);
						h=t*a[ip][iq];
						z[ip] -= h;
						z[iq] += h;
						d[ip] -= h;
						d[iq] += h;
						a[ip][iq]=0.0;
						for (j=0;j<ip;j++)
							rot(a,s,tau,j,ip,j,iq);
						for (j=ip+1;j<iq;j++)
							rot(a,s,tau,ip,j,j,iq);
						for (j=iq+1;j<n;j++)
							rot(a,s,tau,ip,j,iq,j);
						for (j=0;j<n;j++)
							rot(v,s,tau,j,ip,j,iq);
						++nrot;
					}
				}
			}
			for (ip=0;ip<n;ip++) {
				b[ip] += z[ip];
				d[ip]=b[ip];
				z[ip]=0.0;
			}
		}
		throw("Too many iterations in routine jacobi");
	}
	inline void rot(MatDoub_IO &a, const Doub s, const Doub tau, const Int i,
		const Int j, const Int k, const Int l)
	{
		Doub g=a[i][j];
		Doub h=a[k][l];
		a[i][j]=g-s*(h+g*tau);
		a[k][l]=h+s*(g-h*tau);
	}
};

///
/// \brief eigenvalue decomposition helper class (from numerical recipes 3rd)
///
struct Symmeig
{
	Int n;
	MatDoub z;
	VecDoub d,e;
	Bool yesvecs;

	Symmeig(MatDoub_I &a, Bool yesvec=true) : n(a.nrows()), z(a), d(n),
		e(n), yesvecs(yesvec)
	{
		tred2();
		tqli();
		sort();
	}
	Symmeig(VecDoub_I &dd, VecDoub_I &ee, Bool yesvec=true) :
		n(dd.size()), d(dd), e(ee), z(n,n,0.0), yesvecs(yesvec)
	{
		for (Int i=0;i<n;i++) z[i][i]=1.0;
		tqli();
		sort();
	}
	void sort() {
		if (yesvecs)
			eigsrt(d,&z);
		else
			eigsrt(d);
	}
	void tred2();
	void tqli();
	Doub pythag(const Doub a, const Doub b);
};