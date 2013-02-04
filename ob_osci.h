/* -----------------------------------------------------------------------------

  BrainBay  Version 1.8, GPL 2003-2011, contact: chris@shifz.org
  
  OB_OSCI.H:  contains the OSCI-Object
  the object's propertries are declared and the
  constructor-, make_dialog-, load-, save-, incoming_data-, work-, and destructor-
  methods are implemented here
  
-----------------------------------------------------------------------------*/


#include "brainBay.h"


class OSCIOBJ : public BASE_CL
{
  protected: 
	DWORD dwRead,dwWritten;
	int t;
	  
  public:

	float    input[MAX_EEG_CHANNELS];

	float    pixelbuffer[MAX_EEG_CHANNELS][LEN_PIXELBUFFER];
	int	     prev_pixel[MAX_EEG_CHANNELS];
	float    pixelmem[MAX_EEG_CHANNELS][1024];
	WORD	 newpixels;
	WORD	 signal_pos;

	int      timer;
	int      timercount;
	int		 showgrid;
	int		 showline;
	int		 within;
	int		 group;
	int		 gradual;
	int		 gain;
	int		 mempos;
	int		 redraw;
	int		mysec;
	int		mysec_total;
	int		inc_mysec;
	int		 showseconds;
	int		 drawstart,drawend,periods;
	int		 groupselect;
	float	 laststamp;

	int      top,left,right,bottom;
	COLORREF bkcol;
	HBRUSH   bkbrush;
	COLORREF gridcol;
	HPEN	 gridpen;
	COLORREF captcol;
	HPEN     captpen;
	COLORREF sigcol[MAX_EEG_CHANNELS];
	int		 sigsize[MAX_EEG_CHANNELS];
	HPEN     drawpen[MAX_EEG_CHANNELS];

	
	OSCIOBJ(int num);
	void update_inports(void);
	void session_reset(void);
	void session_pos(long pos);
	void make_dialog(void);
	void load(HANDLE hFile);
	void save(HANDLE hFile);
	void incoming_data(int port, float value);
	void work(void);
	~OSCIOBJ();

   
};
