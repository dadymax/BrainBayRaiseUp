/* -----------------------------------------------------------------------------

  BrainBay  Version 1.8, GPL 2003-2011, contact: chris@shifz.org
  
  MODULE: OB_DEBOUNCE.CPP:  functions for the Debounce-Object
  Author: Chris Veigl

  This Object filters sudden changes to INVALID_VALUE. the number of considered
  samples can be selected.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; See the
  GNU General Public License for more details.

-----------------------------------------------------------------------------*/

#include "brainBay.h"
#include "ob_debounce.h"

DEBOUNCEOBJ::DEBOUNCEOBJ(int num) : BASE_CL()
{
	outports = 1;
	inports = 1;
	width=75;
	strcpy(out_ports[0].out_name,"out");
	strcpy(in_ports[0].in_name,"in");
	dtime=0;active=0;
	count_time=0;
}
	
void DEBOUNCEOBJ::make_dialog(void)
{
	display_toolbox(hDlg=CreateDialog(hInst, (LPCTSTR)IDD_DEBOUNCEBOX, ghWndStatusbox, (DLGPROC)DebounceDlgHandler));
}

void DEBOUNCEOBJ::load(HANDLE hFile) 
{
	load_object_basics(this);
    load_property("dtime",P_INT,&dtime);
}

void DEBOUNCEOBJ::save(HANDLE hFile) 
{
	save_object_basics(hFile, this);
    save_property(hFile,"dtime",P_INT,&dtime);
}
	


  	
void DEBOUNCEOBJ::incoming_data(int port, float value)
{
	out=value;

	if ((!active) && (old_value==INVALID_VALUE)&&(value!=INVALID_VALUE))
		active=1;

	if ((active==1)&&(value==INVALID_VALUE)) {count_time=0; active=2; }

	if (active==2)
	{
		if (count_time++>dtime)  active=0; else	out=INVALID_VALUE;
	}  

	old_value=value;
}
	
void DEBOUNCEOBJ::work(void)
{
 	pass_values(0, out);
}


DEBOUNCEOBJ::~DEBOUNCEOBJ() {}

LRESULT CALLBACK DebounceDlgHandler(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	DEBOUNCEOBJ * st;
	
	st = (DEBOUNCEOBJ *) actobject;
	if ((st==NULL)||(st->type!=OB_DEBOUNCE)) return(FALSE);
	
	switch( message )
	{
		case WM_INITDIALOG:

				SetDlgItemInt(hDlg, IDC_DTIME, st->dtime, FALSE);
                
				break;		
		case WM_CLOSE:
			    EndDialog(hDlg, LOWORD(wParam));
				return TRUE;
			break;
		case WM_COMMAND:
			switch (LOWORD(wParam)) 
			{
				case IDC_DTIME:
					st->dtime=GetDlgItemInt(hDlg,IDC_DTIME,NULL,0);
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




