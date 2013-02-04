 /* ----------------------------------------------------------------------------------

  BrainBay  -  OpenSource Application for realtime BodySignalProcessing & HCI
               with the OpenEEG hardware, GPL 2003-2010
			   
  Author: Chris Veigl, contact: chris@shifz.org
  
  Co-Authors:
		 Jeremy Wilkerson (Modules: AND, OR, NOT, WAV, CORELLATION, EVALUATOR)
		 Lester John (Module MATLAB-transfer)
		 Stephan Gerhard (QDS parser)
		 Franz Stobl ( NIA support )

  Credits: Jim Peters (digital filter works), Jeff Molofee (OpenGL-tutorial), John Roark (SkinDialog)
  		   AllenD (COM-Port control), Aleksandar B. Samardz (Expression Evaluator Library)
		   Craig Peacock (PortTalk IO driver)

  the used non-standard Libraries are:

  Multimedia and OpenGL: winmm.lib opengl32.lib glu32.lib vfw32.lib glaux.lib
  SDL (Simple Direct Media Layer): SDL.lib SDL_net.lib SDL_sound.lib modplug.lib 
  OpenCV - Intels's Computer Vision Library: cv.lib cvcam.lib cxcore.lib highgui.lib
  Matlab Engine (only in special Matlab Release): libeng.lib libmx.lib 
  Jim Peters's Filter Library: fidlib.lib (http://uazu.net)
  Skinned Dialog by John Roark: skinstyle.lib (http://www.codeproject.com/dialog/skinstyle.asp)
  GNU LibMatheval by Aleksandar B. Samardz: matheval.lib (http://www.gnu.org/software/libmatheval)
  Video Input Camera Capture Library: http://muonics.net/school/spring05/videoInput/

  Project-Site: http://brainbay.lo-res.org
  Link to the OpenEEG-Project: http://openeeg.sf.net
  
 
  
 -------------------------------------------------------------------------------------
   
  MODULE:  brainBay.cpp  - Main Module
		creates the Main Window, handle Menu events, 
		provides a mouse-oriented interface for moving and linking objects
   
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; See the
  GNU General Public License for more details.

  
-------------------------------------------------------------------------------------*/




#include "brainBay.h"
#include "ob_osci.h"
#include "ob_skindialog.h"
#include "ob_neurobit.h"

LRESULT CALLBACK AboutDlgHandler( HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam );
LRESULT CALLBACK SCALEDlgHandler( HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam );
LRESULT CALLBACK COLORDlgHandler(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);


//extern int lk_work (void);
extern NEUROBITOBJ * NB_OBJ;


int check_keys(void)
{
    static int mode=0;

	if (mode)
	{ if (!GetAsyncKeyState(116) && !GetAsyncKeyState(117) && !GetAsyncKeyState(118) && !GetAsyncKeyState(119))
	    mode=0;   else return(0);
	}

	if (GetAsyncKeyState(118))
	{mode=1; SendMessage(ghWndMain,WM_COMMAND,IDM_PLAY,0);}
	else if (GetAsyncKeyState(119))
	{mode=1;SendMessage(ghWndMain,WM_COMMAND,IDM_STOP,0);}
	else if (GetAsyncKeyState(116))
	{mode=1;	SendMessage(ghWndStatusbox,WM_COMMAND,IDC_DESIGN,0);}
	else if (GetAsyncKeyState(117))
	{mode=1;	SendMessage(ghWndStatusbox,WM_COMMAND,IDC_HIDE,0);}

	return(0);

}

/* 

// used to convert capture files 

int conv_file(void)
{

    HANDLE hFile,hFile2;
    BOOL bSuccess = FALSE;
	DWORD dwRead,dwWritten;
	unsigned char buffer[3];
	unsigned char buffer2[3];
	int num=0;


	 hFile = CreateFile( (LPCTSTR) "c:\\capture3.txt", GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
     if (hFile == INVALID_HANDLE_VALUE)	 	printf ("file1 open error\n");
	 hFile2 = CreateFile((LPCTSTR) "c:\\cp.txt", GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
     if (hFile2 == INVALID_HANDLE_VALUE)	 	printf ("file1 open error\n");

    if ((hFile != INVALID_HANDLE_VALUE) && (hFile2 != INVALID_HANDLE_VALUE))
    {
       do
	   {
         ReadFile(hFile, (unsigned char *) buffer , 2, &dwRead, NULL);
		 if (dwRead==2)
		 {
			 if ((buffer[0]>='0') && (buffer[0]<='9'))  buffer2[0]=(buffer[0]-'0') * 16;
			 else if ((buffer[0]>='A') && (buffer[0]<='F'))  buffer2[0]=(buffer[0]-'A'+10) * 16;
			 
			 if ((buffer[1]>='0') && (buffer[1]<='9'))  buffer2[0]+=(buffer[1]-'0');
			 else if ((buffer[1]>='A') && (buffer[1]<='F'))  buffer2[0]+=(buffer[1]-'A'+10);
	         
			 printf ("%d,",buffer2[0]);

	  	      WriteFile(hFile2, (unsigned char * )buffer2, 1, &dwWritten, NULL);
			  num++;
		 }
		 printf ("read %d bytes\n",num);

	   }
	   while (dwRead==2);
	}
	else 	printf ("file open error\n");

        			 
    CloseHandle(hFile);
    CloseHandle(hFile2);

	printf("done !\n");
    return(0);

}

*/


//-----------------------------------------------------------------------
//  FUNCTION WinMain
//  PURPOSE:  application entry point, register MainWindow class
//-----------------------------------------------------------------------
int APIENTRY WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,
                     int       nCmdShow )
{
	MSG msg;
	hInst=hInstance;

	// conv_file();

	init_path();
	register_classes(hInstance);

    if(!(ghWndMain=CreateWindow("brainBay_Class", "BrainBay", 
     	 WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN, 10, 20, 900, 520, NULL, NULL, hInstance, NULL)))
        critical_error("can't create main Window");
    else {GLOBAL.left=20;GLOBAL.top=20;GLOBAL.right=900;GLOBAL.bottom=520; }
    ShowWindow( ghWndMain, SW_SHOWNORMAL );
    UpdateWindow( ghWndMain );


	create_logfile();
	write_logfile("BrainBay start.");
	GlobalInitialize();

	ghWndStatusbox=CreateDialog(hInst, (LPCTSTR)IDD_STATUSBOX, ghWndMain, (DLGPROC)StatusDlgHandler); 

	if(!(ghWndDesign=CreateWindow("Design_Class", "Design", WS_CLIPSIBLINGS | WS_CAPTION  | WS_THICKFRAME | WS_CHILD | WS_HSCROLL | WS_VSCROLL ,GLOBAL.design_left, GLOBAL.design_top, GLOBAL.design_right-GLOBAL.design_left, GLOBAL.design_bottom-GLOBAL.design_top, ghWndMain, NULL, hInst, NULL))) 
	    report_error("can't create Design Window");
	else 
	{
		SCROLLINFO si;
        ZeroMemory(&si, sizeof(si));
	    si.cbSize = sizeof(si);
		si.fMask = SIF_TRACKPOS|SIF_RANGE|SIF_TRACKPOS;
	    GetScrollInfo(ghWndDesign, SB_HORZ, &si);
		si.nMax=5000; si.nMin=0; si.nPos=0; si.nTrackPos=0;
		SetScrollInfo(ghWndDesign, SB_HORZ, &si,TRUE);
	    GetScrollInfo(ghWndDesign, SB_VERT, &si);
		si.nMax=5000; si.nMin=0; si.nPos=0; si.nTrackPos=0;
		SetScrollInfo(ghWndDesign, SB_VERT, &si,TRUE);
		ShowWindow( ghWndDesign, TRUE ); UpdateWindow( ghWndDesign ); 
	}

	if (GLOBAL.startup) 
	{
		if (!load_configfile(GLOBAL.configfile)) report_error("Could not load Config File");
		//else sort_objects();
	}

	update_status_window();

	
////////////
	

	

	// main message loop

	while (TRUE)
	{

		check_keys();

  		if(!GetMessage(&msg, NULL, 0, 0))	break;

		TranslateMessage(&msg);
		DispatchMessage(&msg);

		if(!PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE))
		{  
			if (DRAW.particles) InvalidateRect(ghWndAnimation,NULL,FALSE);
		}
		
	}
	return msg.wParam;
}



//
//  function: MainWndHandler(HWND, unsigned, WORD, LONG)
//
//  purpose:	make initialisations (globals, Def-TTY settings)
//				create Toolbox Dialog
//
//			    process  Menu Selections
//					load/save Config
//					play/capture Archive 
//					open midi-device
//					change com settings
//					connect/disconnect to EEG-amp
//					load/save Config
//
//				scale and paint the channel-oscilloscope
//
//
//
LRESULT CALLBACK MainWndHandler(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	char sztemp[256];
    
	switch( message ) 
	{

	 case WM_CREATE:
		break;

	 case WM_ENABLE:
		 if ((wParam==TRUE) && (NB_OBJ != NULL)) 
		 {
			 NB_OBJ->update_channelinfo();
			 NB_OBJ=NULL;
		 }
		 break;

	 case WM_COMMAND:
			wmId    = LOWORD(wParam); 
			wmEvent = HIWORD(wParam); 
			// Men�auswahlen analysieren:
			switch( wmId ) 
			{

				case IDM_NEWCONFIG:
					stop_timer();
					TTY.read_pause=1;
					//BreakDownCommPort();
					CAPTFILE.do_read=0;
					close_captfile();

					if (ghWndAnimation!=NULL) SendMessage(ghWndAnimation,WM_CLOSE,0,0);
					ghWndAnimation=NULL;
					close_toolbox();

	 		   	    write_logfile("new config: deleting all objects.");
					while (GLOBAL.objects>0)
						free_object(0);

					deviceobject=NULL;
					GLOBAL.showdesign=TRUE;
					ShowWindow(ghWndDesign,TRUE);
				    SetWindowPos(ghWndDesign,HWND_TOP,0,0,0,0,SWP_DRAWFRAME|SWP_NOMOVE|SWP_NOSIZE);
					SetDlgItemText(ghWndStatusbox,IDC_DESIGN,"Hide Design"); 
					GLOBAL.hidestatus=FALSE;
					ShowWindow(ghWndStatusbox,TRUE);
				    SetWindowText(ghWndMain,"BrainBay");
					GLOBAL.configfile[0]=0;
					init_system_time();	PACKET.readstate=0;
					GLOBAL.session_length=0;
					SetDlgItemText(ghWndStatusbox,IDC_STATUS,"ready.");
					SetDlgItemInt(ghWndStatusbox,IDC_SAMPLINGRATE,PACKETSPERSECOND,0);
					SetDlgItemText(ghWndStatusbox,IDC_STATUS,"Configuration loaded");
					SetDlgItemText(ghWndStatusbox,IDC_TIME,"0.0");
					SetDlgItemText(ghWndStatusbox,IDC_JUMPPOS,"0.0");
					SetDlgItemText(ghWndStatusbox,IDC_SESSLEN,"0.0");
 					SendMessage(GetDlgItem(ghWndStatusbox,IDC_SESSIONPOS),TBM_SETPOS,TRUE,(LONG)0);
					SendMessage(GetDlgItem(ghWndStatusbox,IDC_SESSIONPOS),TBM_SETSELEND,TRUE,(LONG)0);
					SendMessage(GetDlgItem(ghWndStatusbox,IDC_SESSIONPOS),TBM_SETSELSTART,TRUE,(LONG)0);

 					InvalidateRect(ghWndDesign,NULL,TRUE);
					InvalidateRect(ghWndMain,NULL,TRUE);
					break;

				case IDM_SAVECONFIG:
					{ 
						char configfilename[MAX_PATH];
						int save_toolbox=GLOBAL.showtoolbox;

						close_toolbox();
						GLOBAL.showtoolbox=save_toolbox;
						strcpy(configfilename,GLOBAL.resourcepath); 
						if (GLOBAL.configfile[0]==0)
  						  strcat(configfilename,"CONFIGURATIONS\\*.con");
						else strcpy(configfilename,GLOBAL.configfile);
						if (open_file_dlg(hWnd,configfilename, FT_CONFIGURATION, OPEN_SAVE))
						{
						   if (!save_configfile(configfilename))  report_error("Could not save Config File");
						   else
						   {
							  char * d_name,new_name[80];
		  	 		   	      write_logfile("configruation saved.");

							  d_name=configfilename;
							  while (strstr(d_name,"\\")) d_name=strstr(d_name,"\\")+1;
							  strcpy(new_name,"BrainBay - ");strcat(new_name,d_name);
							  SetWindowText(ghWndMain,new_name);
						   }
						}
						if (GLOBAL.showtoolbox!=-1)
						{	
							actobject=objects[GLOBAL.showtoolbox];
							actobject->make_dialog();
						}
					}
					break;
				case IDM_LOADCONFIG:
					{ 
						char configfilename[MAX_PATH];
						close_toolbox();
						strcpy(configfilename,GLOBAL.resourcepath); 
						strcat(configfilename,"CONFIGURATIONS\\*.con");
						if (open_file_dlg(hWnd,configfilename, FT_CONFIGURATION, OPEN_LOAD))
						{
			 		   	    write_logfile("load config: free existing objects.");
							if (!load_configfile(configfilename)) report_error("Could not load Config File");
							else sort_objects();
						}
					
					}
					break;

				case IDM_ABOUT:
					close_toolbox();
				    DialogBox(hInst, (LPCTSTR)IDD_ABOUTBOX, hWnd, (DLGPROC)AboutDlgHandler);
					break;

				case IDM_HELP:
					{
						char tmpfile [250];
						close_toolbox();
						strcpy(tmpfile,GLOBAL.resourcepath);
						strcat(tmpfile,"BrainBay-user_manual.pdf");
						ShellExecute(0, "open", tmpfile, NULL, NULL, SW_SHOWNORMAL);
					}
					break;
				case IDM_VIEWLASTLOGFILE:
					{
						char tmpfile [250];
						close_toolbox();
						strcpy(tmpfile,GLOBAL.resourcepath);
						strcat(tmpfile,"bbay.log");
						ShellExecute(0, "open", tmpfile, NULL, NULL, SW_SHOWNORMAL);
					}

					break;
				case IDM_EXIT:	
					DestroyWindow( hWnd );
				    break;
                
				case IDM_PLAY:	
						SendMessage(ghWndStatusbox,WM_COMMAND, IDC_RUNSESSION,0);
				    break;
				case IDM_STOP:			
						SendMessage(ghWndStatusbox,WM_COMMAND, IDC_STOPSESSION,0);
				    break;

				case IDM_NEUROSERVER:
						strcpy(sztemp,GLOBAL.resourcepath); 
						strcat(sztemp,"NETWORK\\nsd.exe");
						ShellExecute(hWnd, "open", sztemp, NULL, NULL, SW_SHOWNORMAL);
						break;
				case IDM_READEDF:
						{
							char edffilename[MAX_PATH];
							close_toolbox();
							strcpy(edffilename,GLOBAL.resourcepath); 
							strcat(edffilename,"ARCHIVES\\*.edf");
							if (open_file_dlg(hWnd,edffilename, FT_EDF, OPEN_LOAD))
							{
								strcpy(sztemp,GLOBAL.resourcepath); 
								strcat(sztemp,"NETWORK\\readedf.exe");
								ShellExecute(hWnd, NULL, sztemp, edffilename, NULL, SW_SHOWNORMAL);
							}
						}
						break;
				case IDM_EDITCOLORS:
					close_toolbox();
					display_toolbox(CreateDialog(hInst, (LPCTSTR)IDD_EDITCOLORBOX, ghWndStatusbox, (DLGPROC)COLORDlgHandler));
					break;
				case IDM_EDITSCALES:
					close_toolbox();
					display_toolbox(CreateDialog(hInst, (LPCTSTR)IDD_EDITSCALEBOX, ghWndStatusbox, (DLGPROC)SCALEDlgHandler));
					break;
				case IDM_SETTINGS:
                     if (ghWndSettings==NULL) ghWndSettings=CreateDialog(hInst, (LPCTSTR)IDD_SETTINGSBOX, ghWndStatusbox, (DLGPROC)SETTINGSDlgHandler);
					 else SetForegroundWindow(ghWndSettings);
					break;
				case IDM_DEVICESETTINGS:
					if (deviceobject) 
					{
						close_toolbox();
						actobject=deviceobject;
	//					GLOBAL.showtoolbox=find_object(devicebox);
						actobject->make_dialog(); 
						if (actobject->displayWnd) 
							SetWindowPos(actobject->displayWnd,HWND_TOP,0,0,0,0,SWP_DRAWFRAME|SWP_NOMOVE|SWP_NOSIZE);
					} else report ("No Amplifier Device present in the design");
					break;
				case IDM_INSERTMODEEG: 
					if (!count_objects(OB_EEG)) create_object(OB_EEG); 
					break;
				case IDM_INSERTMIDI: create_object(OB_MIDI); 
					break;
				case IDM_INSERTSPECTRUM: create_object(OB_FFT);
					break;	
				case IDM_INSERTTHRESHOLD: create_object(OB_THRESHOLD); 
					break;
				case IDM_INSERTFILTER:	create_object(OB_FILTER);						
					break;	
				case IDM_INSERTMAGNITUDE:create_object(OB_MAGNITUDE);						
					break;	
				case IDM_INSERTPARTICLE:create_object(OB_PARTICLE);						
					break;	
				case IDM_INSERTOSCI:create_object(OB_OSCI);						
					break;	
				case IDM_INSERTTRANSLATE:create_object(OB_TRANSLATE);						
					break;
				case IDM_INSERTSIGNAL:create_object(OB_SIGNAL);
					break;
				case IDM_INSERTAND:create_object(OB_AND);
					break;
				case IDM_INSERTOR:create_object(OB_OR);
					break;
				case IDM_INSERTNOT:create_object(OB_NOT);
					break;
				case IDM_INSERTWAV:
					if (!count_objects(OB_WAV)) create_object(OB_WAV);
					else report_error("Currently only one Sound player is supported.");
					break;
				case IDM_INSERTTCPRECEIVER:create_object(OB_TCP_RECEIVER);
					break;
				case IDM_INSERTDOKU:create_object(OB_DOKU);
					break;
                case IDM_INSERTEVAL:create_object(OB_EVAL);
					break;
				case IDM_INSERTAVI:create_object(OB_AVI);
					break;
				case IDM_INSERTAVERAGE:create_object(OB_AVERAGE);
					break;
				case IDM_INSERTCORR:create_object(OB_CORR);
					break;
				case IDM_INSERTEDFWRITER:create_object(OB_EDF_WRITER);
					break;
				case IDM_INSERTTCPSENDER:create_object(OB_TCP_SENDER);
					break;
				case IDM_INSERTEDFREADER:create_object(OB_EDF_READER);
					break;
				case IDM_INSERTCOMPARE:create_object(OB_COMPARE);
					break;
				case IDM_INSERTBALLGAME:create_object(OB_BALLGAME);
					break;
				case IDM_INSERTMIXER4:create_object(OB_MIXER4);
					break;
				case IDM_INSERTMOUSE:create_object(OB_MOUSE);
					break;
				case IDM_INSERTERPDETECT:create_object(OB_ERPDETECT);
					break;
				case IDM_INSERTCOM_WRITER:create_object(OB_COM_WRITER);
					break;
				case IDM_INSERTCAM:
					if (!count_objects(OB_CAM)) create_object(OB_CAM);
					break;
				case IDM_INSERTINTEGRATE:create_object(OB_INTEGRATE);
					break;
				case IDM_INSERTDEBOUNCE:create_object(OB_DEBOUNCE);
					break;
				case IDM_INSERTSAMPLE_HOLD:create_object(OB_SAMPLE_HOLD);
					break;
				case IDM_INSERTCONSTANT:create_object(OB_CONSTANT);
					break;
				case IDM_INSERTMATLAB:create_object(OB_MATLAB);
					break;
				case IDM_INSERTCOUNTER:create_object(OB_COUNTER);
					break;
				case IDM_INSERTSKINDIALOG:
					if (!count_objects(OB_SKINDIALOG)) create_object(OB_SKINDIALOG);
					break;
				case IDM_INSERTFILE_WRITER:create_object(OB_FILE_WRITER);
					break;
				case IDM_INSERTDEVIATION:create_object(OB_DEVIATION);
					break;
				case IDM_INSERTMCIPLAYER:create_object(OB_MCIPLAYER);
					break;
				case IDM_INSERTKEYSTRIKE:create_object(OB_KEYSTRIKE);
					break;
				case IDM_INSERTPEAKDETECT:create_object(OB_PEAKDETECT);
					break;
				case IDM_INSERTSPELLER:create_object(OB_SPELLER);
					break;
				case IDM_INSERTMARTINI:create_object(OB_MARTINI);
					break;
				case IDM_INSERTFILE_READER:create_object(OB_FILE_READER);
					break;
				case IDM_INSERTPORT_IO:create_object(OB_PORT_IO);
					break;
				case IDM_INSERTARRAY3600:create_object(OB_ARRAY3600);
					break;
				case IDM_INSERTCOMREADER:create_object(OB_COMREADER);
					break;
				case IDM_INSERTOPTIMA:
					if (!count_objects(OB_NEUROBIT)) create_object(OB_NEUROBIT);
					break;
				case IDM_INSERTMIN:create_object(OB_MIN);
					break;
				case IDM_INSERTMAX:create_object(OB_MAX);
					break;
				case IDM_INSERTROUND:create_object(OB_ROUND);
					break;
				case IDM_INSERTDIFFERENTIATE:create_object(OB_DIFFERENTIATE);
					break;
				case IDM_INSERTDELAY:create_object(OB_DELAY);
					break;
				case IDM_INSERTLIMITER:create_object(OB_LIMITER);
					break;

				case IDM_COPY:
					if (actobject)
					{
						copy_object=actobject;
						if ((copy_object->type!=OB_EEG)&&(copy_object->type!=OB_WAV)&&(copy_object->type!=OB_CAM)
							&&(copy_object->type!=OB_SKINDIALOG)&&(copy_object->type!=OB_NEUROBIT))
						{
						    HANDLE hFile;
							char tmpfile[256];
							create_object(copy_object->type);

							strcpy(tmpfile,GLOBAL.resourcepath); 
							strcat(tmpfile,"tmp_copy.con");
							hFile = CreateFile(tmpfile, GENERIC_WRITE, 0, NULL,CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
							if(hFile != INVALID_HANDLE_VALUE)
							{
									copy_object->save(hFile);
									save_property(hFile,"end Object",P_END,NULL);
									CloseHandle(hFile);
							}
							hFile = CreateFile(tmpfile, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
							if(hFile != INVALID_HANDLE_VALUE)
							{
									load_next_config_buffer(hFile);
									actobject->load(hFile);
									CloseHandle(hFile);
									DeleteFile(tmpfile);
							}
							actobject->xPos+=10; actobject->yPos+=10;
						}		
						close_toolbox();
					}
					break;
								
				default:
				   return DefWindowProc( hWnd, message, wParam, lParam );
			}
			break;

		case WM_ACTIVATE:
				{
/*				 char t[50];

				 static int cn=0;
				 cn++;
				 sprintf(t,"%d:%d,%d",cn,HIWORD(lParam),LOWORD(lParam)); //==WA_CLICKACTIVE))
				 SendDlgItemMessage(ghWndStatusbox,IDC_LIST2, LB_ADDSTRING, 0, (LPARAM) t);
				 SendDlgItemMessage(ghWndStatusbox,IDC_LIST2, LB_SETCURSEL, SendDlgItemMessage(ghWndStatusbox,IDC_LIST2, LB_GETCOUNT, 0, 0)-1, 0);
				 UpdateWindow(ghWndStatusbox);
*/
			     if ((LOWORD(lParam)==WA_CLICKACTIVE) || (HIWORD(lParam)==WA_CLICKACTIVE))
				 	SetWindowPos(ghWndMain,HWND_TOP,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
				  SetFocus(ghWndMain);
			}
			break;


		case WM_ACTIVATEAPP: 

			{
/*				 char t[50];
				 static int cn=0;
				 cn++;
				 sprintf(t,"%d:%d,%d",cn,HIWORD(lParam),LOWORD(lParam)); //==WA_CLICKACTIVE))
				 SendDlgItemMessage(ghWndStatusbox,IDC_LIST2, LB_ADDSTRING, 0, (LPARAM) t);
				 SendDlgItemMessage(ghWndStatusbox,IDC_LIST2, LB_SETCURSEL, SendDlgItemMessage(ghWndStatusbox,IDC_LIST2, LB_GETCOUNT, 0, 0)-1, 0);
				 UpdateWindow(ghWndStatusbox); */

//			     if ((LOWORD(lParam)==1828) || (LOWORD(lParam)==964))
				 	//SetWindowPos(ghWndMain,HWND_TOP,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
					SetWindowPos(ghWndMain,0,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE);
//				 return DefWindowProc( hWnd, message, wParam, lParam );
			}

			break;
		case WM_KEYDOWN:
		    if (lParam==KEY_DELETE )
			  SendMessage(ghWndDesign, message,wParam,lParam);
			 break;

		case WM_SIZE:
			 if (wParam== SIZE_MAXIMIZED) 
			 {
				 GLOBAL.main_maximized=lParam;
				 ShowWindow(ghWndStatusbox,TRUE);
				 GLOBAL.hidestatus=FALSE;
				 if (GLOBAL.session_length==0)
  				    SetWindowPos(ghWndStatusbox, ghWndMain, 4, HIWORD(lParam)+15, 
			                   LOWORD(lParam)-8,HIWORD(lParam), 0);
				 else SetWindowPos(ghWndStatusbox, ghWndMain, 4, HIWORD(lParam)-20, 
			                   LOWORD(lParam)-8,HIWORD(lParam), 0);
			 }
			 else if (wParam== SIZE_RESTORED) 
			 {
				WINDOWPLACEMENT  wndpl;
				GetWindowPlacement(ghWndMain, &wndpl);
				GLOBAL.top=wndpl.rcNormalPosition.top;
				GLOBAL.left=wndpl.rcNormalPosition.left;
				GLOBAL.right=wndpl.rcNormalPosition.right;
				GLOBAL.bottom=wndpl.rcNormalPosition.bottom;
				GLOBAL.main_maximized=0;
				update_status_window();

			 }
			 else if (wParam== SIZE_MINIMIZED) 
			 {
				 ShowWindow(ghWndStatusbox,FALSE);
				 GLOBAL.hidestatus=TRUE;
			 }
			break;
    	case WM_MOVE:
			{
				WINDOWPLACEMENT  wndpl;
				GetWindowPlacement(ghWndMain, &wndpl);
				GLOBAL.top=wndpl.rcNormalPosition.top;
				GLOBAL.left=wndpl.rcNormalPosition.left;
				GLOBAL.right=wndpl.rcNormalPosition.right;
				GLOBAL.bottom=wndpl.rcNormalPosition.bottom;
				update_status_window();
			}
			break;
		//case WM_PAINT:
		    //	break;
		case WM_DESTROY:
			actobject=0;
			GlobalCleanup();
			PostQuitMessage( 0 );
			break;
		case WM_INPUT:
			if ((TTY.CONNECTED) && (!GLOBAL.loading))
				ReadNIA( wParam, lParam);			// NIA Daten vorhanden: auslesen
				return(TRUE);
		default:
			return DefWindowProc( hWnd, message, wParam, lParam );
   }
   return 0;
}
