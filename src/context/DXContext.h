#pragma once

#include <windows.h>
#include <string>
#include <utility>
#include "IContextProcessable.h"
#include <vector>

using namespace std;

static LRESULT CALLBACK WndProc( HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam );


// =======================================================================================
//                                      DXContext
// =======================================================================================

///---------------------------------------------------------------------------------------
/// \brief	Window context for DirectX
///        
/// # DXContext
/// 
/// 17-4-2013 Jarl Larsson
///---------------------------------------------------------------------------------------

class DXContext
{
public:
	DXContext(HINSTANCE p_hInstance, const string& p_title,
		int p_width, int p_height);
	virtual ~DXContext();
	HWND getWindowHandle();
	static DXContext* getInstance();

	void close();

	///-----------------------------------------------------------------------------------
	/// Resize the window
	/// \param p_w
	/// \param p_h
	/// \param p_update set to true to force an update, 
	///					if an update has already been done by windows, set to false.
	/// \return void
	///-----------------------------------------------------------------------------------
	void resize(int p_w, int p_h, bool p_update);

	///-----------------------------------------------------------------------------------
	/// Change the window title string
	/// \param p_title
	/// \return void
	///-----------------------------------------------------------------------------------
	void setTitle(const string& p_title);

	///-----------------------------------------------------------------------------------
	/// Update the window with the store title
	/// \param p_appendMsg Optional message string to append to title
	/// \return void
	///-----------------------------------------------------------------------------------
	void updateTitle(const string& p_appendMsg="");

	///-----------------------------------------------------------------------------------
	/// Whether a closedown was requested
	/// \return bool
	///-----------------------------------------------------------------------------------
	bool closeRequested() const;

	///-----------------------------------------------------------------------------------
	/// Returns true if window has been resized. Dirty bit is reset upon call if true.
	/// \return bool
	///-----------------------------------------------------------------------------------
	bool isSizeDirty();

	pair<int,int> getSize();

	///-----------------------------------------------------------------------------------
	/// Add sub processor that needs to subscribe to windows events
	/// \return void
	///-----------------------------------------------------------------------------------
	void addSubProcess(IContextProcessable* p_proc);

	///-----------------------------------------------------------------------------------
	/// Remove matching sub processor
	/// \return void
	///-----------------------------------------------------------------------------------
	void removeSubProcessEntry(const IContextProcessable* p_proc);

	///-----------------------------------------------------------------------------------
	/// Executes all sub processes that need window events
	/// \return void
	///-----------------------------------------------------------------------------------
	bool runSubProcesses(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
protected:
private:
	bool m_closeFlag;
	bool m_sizeDirty;
	string m_title;
	int m_width;
	int m_height;
	HINSTANCE	m_hInstance;
	HWND		m_hWnd;
	static DXContext* m_instance;
	//
	std::vector<IContextProcessable*> m_processors;
};