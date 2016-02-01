/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/

#ifndef DISPWIDGET_JURY_TRANSMITTER
#define DISPWIDGET_JURY_TRANSMITTER


#include "stdafx.h"

//! class of qt widget
    /*!
    this is the QWidget pf the jury transmitter
    */
class DisplayWidget : public QWidget
{
    Q_OBJECT

    public:
        /*! constructor for the widget
        @param numberSections the number of the section which are selectable
        @param parent the parent widget
        */
        DisplayWidget(QWidget* parent);
        ~DisplayWidget() {};
    public slots:        
        /*! sets the current state of the driver
        @param state state to be sent
        @param entryId current entry to be sent
        */
        void OnDriverState(int state, int entryId);
        
        /*! appends a new line to the textfield
        @param text new text for the log line
        */
        void OnAppendText(QString text);

    public:
        /*! this function sets the given sectorList to m_sectorList of the Widget
        @param sectorList sectorList to be printed*/
        void SetManeuverList(std::vector<tSector> sectorList);

        /*! this function sets the given sectorList to the log field*/
        void ShowManeuverList();
        /*! this function fills combobox */
        void FillComboBox();
        
    private slots:
        /*! slot for start button*/
        void OnStartButtonClicked();
        /*! slot for stop button*/
        void OnStopButtonClicked();
        /*! slot for request ready button*/
        void OnStopRequestReadyClicked();
        /*! slot for the Not aus*/
        void OnNotAusButtonClicked();
        /*! slot which will be executed when combobox is changed*/
        void OnComboActionBoxChanged(int index);
        /*! slot which will be executed when combobox is changed*/
        void OnComboSectionBoxChanged(int index);
    signals:
        /*! signal to cJuryModule for Not Aus*/
        void sendNotAus();
        /*! signal to cJuryModule for Stopping Not Aus*/
        void sendStopNotAus();
        /*! signal to cJuryModule for start defined section
        @param entryId new id of maneuver
        */
        void sendStart(tInt16 entryId);
        /*! signal to cJuryModule for stop defined section
        @param entryId new id of maneuver
        */
        void sendStop(tInt16 entryId);
        /*! signal to cJuryModule for ready request to driver
        @param entryId new id of maneuver
        */
        void sendReadyRequest(tInt16 entryId);
        
    private:
        /*! the main widget */
        QWidget* m_pWidget;
        
        /*! the button for the Not Aus */
        QPushButton *m_pNotAusButton;

        /*! the button to start in the given the section */
        QPushButton *m_pStartButton;
        
        /*! the button to stop the car in the section */
        QPushButton *m_pStopButton;

        /*! the button to request the state of the car in the section */
        QPushButton *m_pRequestButton;
        
        /*! combo box for selection the action */
        QComboBox *m_comboActionBox;

        /*! combobox for selected section*/
        QComboBox* m_comboSectionBox;
        
        /*! label for current maneuver id of driver*/
        QLabel *m_driverManeuverID;
        
        /*! rect to show the state of the driver*/
        QLabel *m_stateLabel;
        
        /*! field to show the measured times of the sections*/
        QTextEdit *m_logField;
        
        /*! the main layout of the widget */
        QVBoxLayout* m_mainLayout;
        
        /*! set the error gui state*/
        void setDriverStateError();
        
        /*! set the run gui state*/
        void setDriverStateRun();
        
        /*! set the ready gui state*/
        void setDriverStateReady();

        /*! set the ready gui state*/
        void setDriverStateComplete();
        
        /*! set the maneuver id 
        @param id id of maneuver
        */
        void setDriverManeuverID(tInt16 id);

        std::vector<tSector> m_sectorList;
        
        
};

#endif
