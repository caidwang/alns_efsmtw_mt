/* ALNS_Framework - a framework to develop ALNS based solvers
 *
 * Copyright (C) 2012 Renaud Masson
 *
 * This library is free software; you can redistribute it and/or
 * modify it either under the terms of the GNU Lesser General Public
 * License version 3 as published by the Free Software Foundation
 * (the "LGPL"). If you do not alter this notice, a recipient may use
 * your version of this file under the LGPL.
 *
 * You should have received a copy of the LGPL along with this library
 * in the file COPYING-LGPL-3; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY
 * OF ANY KIND, either express or implied. See the LGPL for
 * the specific language governing rights and limitations.
 *
 * The Original Code is the ALNS_Framework library.
 *
 *
 * Contributor(s):
 *	Renaud Masson
 */

#include "CoolingSchedule_Parameters.h"
#include "../alns/ALNS_Parameters.h"
#include <assert.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include "../lib/tinyxml/tinyxml.h"


using namespace std;

CoolingSchedule_Parameters::CoolingSchedule_Parameters(ALNS_Parameters& alnsPara)
{
	kind = Linear_it;

	expPercentageKept = 0.999;

	setupPercentage = 0.05;

	nbThresholds = 1000;

	maxIt = alnsPara.getMaxNbIterations();

	maxRT = alnsPara.getMaxRunningTime();
}

CoolingSchedule_Parameters::~CoolingSchedule_Parameters()
{
	// Nothing to be done.
}

void CoolingSchedule_Parameters::loadXMLParameters(std::string path)
{
	TiXmlDocument doc(path.c_str());
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		TiXmlElement* root = doc.FirstChildElement( "Parameters" );
		if ( root )
		{
			TiXmlElement* paramCS = root->FirstChildElement( "CoolingSchedule" );
			if(paramCS)
			{
				TiXmlElement* maxRt = paramCS->FirstChildElement( "Kind" );
				if(maxRt)
				{
					stringstream str;
					str << maxRt->GetText();
					string temp;
					str >> temp;
					if(temp == "Linear_it")
					{
						kind = Linear_it;
					}
					else if(temp == "Linear_time")
					{
						kind = Linear_time;
					}
					else if(temp == "Linear_mix")
					{
						kind = Linear_mix;
					}
					else if(temp == "Exponential_it")
					{
						kind = Exponential_it;
					}
					else if(temp == "Exponential_time")
					{
						kind = Exponential_time;
					}
					else if(temp == "Exponential_mix")
					{
						kind = Exponential_mix;
					}
					else
					{
						assert(false);
					}
				}
				TiXmlElement* expPK = paramCS->FirstChildElement( "ExpPercentageKept" );
				if(expPK)
				{
					stringstream str;
					str << expPK->GetText();
					str >> this->expPercentageKept;
				}
				TiXmlElement* setPer = paramCS->FirstChildElement( "SetupPercentage" );
				if(setPer)
				{
					stringstream str;
					str << setPer->GetText();
					str >> this->setupPercentage;
				}
				TiXmlElement* nbTh = paramCS->FirstChildElement( "NbThresholds" );
				if(nbTh)
				{
					stringstream str;
					str << nbTh->GetText();
					str >> this->nbThresholds;
				}
			}
		}
	}
}

void CoolingSchedule_Parameters::loadParameters(string path)
{
	ifstream comingFlow(path.data());
	string line;
	while(getline(comingFlow, line))
	{
		cout << "[DBG] " << line << endl;
		// We check that the line is not a commentary
		if(line.size() > 0 && line.at(0) != '#')
		{
			stringstream str;
			str << line;
			string id;
			string eq;
			str >> id;
			str >> eq;
			if(id == "kind")
			{
				string temp;
				str >> temp;
				if(temp == "Linear_it")
				{
					kind = Linear_it;
				}
				else if(temp == "Linear_time")
				{
					kind = Linear_time;
				}
				else if(temp == "Linear_mix")
				{
					kind = Linear_mix;
				}
				else if(temp == "Exponential_it")
				{
					kind = Exponential_it;
				}
				else if(temp == "Exponential_time")
				{
					kind = Exponential_time;
				}
				else if(temp == "Exponential_mix")
				{
					kind = Exponential_mix;
				}
				else
				{
					assert(false);
				}
			}
			else if(id == "expPercentageKept")
			{
				str >> expPercentageKept;
			}
			else if(id == "setupPercentage")
			{
				str >> setupPercentage;
			}
			else if(id == "nbThresholds")
			{
				str >> nbThresholds;
			}
			else
			{
				assert(false);
			}
		}
	}
}

void CoolingSchedule_Parameters::sanityChecks()
{
	//TODO develop sanity checks.
}

CoolingSchedule_Parameters::CoolingSchedule_Parameters(
		CoolingSchedule_Parameters& p)
{
	kind = p.kind;

	expPercentageKept = p.expPercentageKept;

	setupPercentage = p.setupPercentage;

	nbThresholds = p.nbThresholds;

	maxIt = p.maxIt;

	maxRT = p.maxRT;
}


