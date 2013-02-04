/* -----------------------------------------------------------------------------

  BrainBay  -  Version 1.8, GPL 2003-2011

  MODULE:  OB_OR.CPP
  Authors: Jeremy Wilkerson, Chris Veigl


  This Object performs the OR-operation on it's two Input-Values and presents the
  result at the output-port. FALSE it represented by the constant INVALID_VALUE, TRUE
  is represented by the constand TRUE_VALUE (def: 512.0f )

 This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; See the
  GNU General Public License for more details.

  
-------------------------------------------------------------------------------------*/

#include "brainBay.h"
#include "ob_or.h"


LRESULT CALLBACK OrDlgHandler(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	
	OROBJ * st;
	
	st = (OROBJ *) actobject;
    if ((st==NULL)||(st->type!=OB_OR)) return(FALSE);	

	switch( message )
	{
		case WM_INITDIALOG:
				CheckDlgButton(hDlg, IDC_BINARY, st->binary);
				break;		
		case WM_CLOSE:
			    EndDialog(hDlg, LOWORD(wParam));
				return TRUE;
			break;
		case WM_COMMAND:
			switch (LOWORD(wParam)) 
			{
				case IDC_BINARY:
					st->binary=IsDlgButtonChecked(hDlg,IDC_BINARY);
                    break;

			}
			return TRUE;
			break;

		case WM_SIZE:
		case WM_MOVE:  update_toolbox_position(hDlg);
		break;
		return(TRUE);
	}
	return FALSE;
}




OROBJ::OROBJ(int num) : BASE_CL()
{
	outports = 1;
	inports = 2;
	strcpy(out_ports[0].out_name,"out");
	input1 = INVALID_VALUE;
	input2 = INVALID_VALUE;
	binary=0;
}
	
void OROBJ::make_dialog(void) 
{
	display_toolbox(hDlg=CreateDialog(hInst, (LPCTSTR)IDD_ORBOX, ghWndStatusbox, (DLGPROC)OrDlgHandler));
}


void OROBJ::load(HANDLE hFile) 
{
	load_object_basics(this);
	load_property("binary",P_INT,&binary);
}

void OROBJ::save(HANDLE hFile) 
{
	save_object_basics(hFile, this);
	save_property(hFile,"binary",P_INT,&binary);
}
	
void OROBJ::incoming_data(int port, float value)
{
	if (port == 0)
		input1 = value;
	else if (port == 1)
		input2 = value;
}
	
void OROBJ::work(void)
{
	float value = TRUE_VALUE;
	
	if (!binary)
	{
		if ((input1 == INVALID_VALUE) && (input2 == INVALID_VALUE))
			value = INVALID_VALUE;
	}
	else
	{
		value=(float) ( ((int)input1) | ((int) input2) );
	}
	pass_values(0, value);
}
	
OROBJ::~OROBJ() {}
