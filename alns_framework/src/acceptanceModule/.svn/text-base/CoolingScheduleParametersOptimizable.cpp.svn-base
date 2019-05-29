/* ALNS-framework - a solver for temporal constraint problems
 *
 * Copyright (C) 2013 Renaud Masson
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
 * The Original Code is the ALNS-framework library.
 *
 * File name:
 *  CoolingScheduleParametersOptimizable.cpp
 * Contributor(s):
 *	Renaud Masson
 */

#include "CoolingScheduleParametersOptimizable.h"
#include <sstream>
#include <cstdlib>
#include "../lib/tinyxml/tinyxml.h"

using namespace std;

CoolingSchedule_Parameters_Optimizable::CoolingSchedule_Parameters_Optimizable(ALNS_Parameters& params): CoolingSchedule_Parameters(params) {
	lbExpPercentageKept = expPercentageKept;
	ubExpPercentageKept = expPercentageKept;
	lbSetupPercentage = setupPercentage;
	ubSetupPercentage = setupPercentage;
}

CoolingSchedule_Parameters_Optimizable::CoolingSchedule_Parameters_Optimizable(
		CoolingSchedule_Parameters_Optimizable& p) : CoolingSchedule_Parameters(p)
{
	lbExpPercentageKept = p.lbExpPercentageKept;
	ubExpPercentageKept = p.ubExpPercentageKept;
	lbSetupPercentage = p.lbSetupPercentage;
	ubSetupPercentage = p.ubSetupPercentage;
}

CoolingSchedule_Parameters_Optimizable::~CoolingSchedule_Parameters_Optimizable() {
	// TODO Auto-generated destructor stub
}

std::string CoolingSchedule_Parameters_Optimizable::getHash()
{
	stringstream ss;
	ss << expPercentageKept << "-";
	ss << setupPercentage;
	return ss.str();
}

void CoolingSchedule_Parameters_Optimizable::modifyOneParameter() {
	int paramToMod = rand() % getNbTunableParameters();
	switch(paramToMod)
	{
	case 0:
		modifyExpPercentageKept();
		break;
	case 1:
		modifySetupPercentage();
		break;
	}
}

void CoolingSchedule_Parameters_Optimizable::modifyExpPercentageKept()
{
	expPercentageKept = lbExpPercentageKept + (ubExpPercentageKept-lbExpPercentageKept)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX));
}

void CoolingSchedule_Parameters_Optimizable::modifySetupPercentage()
{
	setupPercentage = lbSetupPercentage + (ubSetupPercentage-lbSetupPercentage)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX));
}

void CoolingSchedule_Parameters_Optimizable::loadAllXMLParameters(
		std::string path)
{
	loadXMLParameters(path);
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
				TiXmlElement* lbExpPK = paramCS->FirstChildElement( "LbExpPercentageKept" );
				if(lbExpPK)
				{
					stringstream str;
					str << lbExpPK->GetText();
					str >> this->lbExpPercentageKept;
				}
				TiXmlElement* ubExpPK = paramCS->FirstChildElement( "UbExpPercentageKept" );
				if(ubExpPK)
				{
					stringstream str;
					str << ubExpPK->GetText();
					str >> this->ubExpPercentageKept;
				}
				TiXmlElement* lbSetPer = paramCS->FirstChildElement( "LbSetupPercentage" );
				if(lbSetPer)
				{
					stringstream str;
					str << lbSetPer->GetText();
					str >> this->lbSetupPercentage;
				}
				TiXmlElement* ubSetPer = paramCS->FirstChildElement( "UbSetupPercentage" );
				if(ubSetPer)
				{
					stringstream str;
					str << ubSetPer->GetText();
					str >> this->ubSetupPercentage;
				}
			}
		}
	}
}
