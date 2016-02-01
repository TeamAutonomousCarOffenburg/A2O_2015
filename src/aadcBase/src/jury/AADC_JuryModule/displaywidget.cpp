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

#include "stdafx.h"
#include "juryEnums.h"
#include "displaywidget.h"



DisplayWidget::DisplayWidget(QWidget* pParent) : QWidget(pParent)
{ 
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);


    // create the elements
    
    QFont font;
    font.setPointSize(20);
    
    
    QLabel *toplabel = new QLabel(this);
    toplabel->setText("AADC - Jury Module");
    toplabel->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    toplabel->setFont(font);
    
    m_pStartButton = new QPushButton(this);
    m_pStartButton->setText("Start");
    m_pStartButton->setFixedWidth(100);

    m_pStopButton = new QPushButton(this);
    m_pStopButton->setText("Stop");
    m_pStopButton->setFixedWidth(100);

    m_pRequestButton = new QPushButton(this);
    m_pRequestButton->setText("Request State");
    m_pRequestButton->setFixedWidth(100);
    
    m_pNotAusButton = new QPushButton(this);
    m_pNotAusButton->setText("Emergency_Stop");
    m_pNotAusButton->setStyleSheet("background-color:yellow");
    m_pNotAusButton->setMinimumSize ( 220, 30 );
    font.setPointSize(14);
    m_pNotAusButton->setFont(font);
    
    QHBoxLayout *hboxManeuverList = new QHBoxLayout();
    m_comboActionBox = new QComboBox(this);
    m_comboSectionBox = new QComboBox(this);

    m_comboActionBox->setMaximumWidth(40);
    m_comboSectionBox->setMaximumWidth(40);
    hboxManeuverList->addWidget(new QLabel(QString("Maneuver Index")));
    hboxManeuverList->addWidget(m_comboActionBox);
    hboxManeuverList->addWidget(new QLabel(QString("Section Index:")));
    hboxManeuverList->addWidget(m_comboSectionBox);
    
    //create the controls for the sections
    
    QVBoxLayout *vboxManeuverControlButtons = new QVBoxLayout();    
    vboxManeuverControlButtons->addWidget(m_pRequestButton,  0, Qt::AlignCenter);
    vboxManeuverControlButtons->addWidget(m_pStartButton, 0, Qt::AlignCenter);
    vboxManeuverControlButtons->addWidget(m_pStopButton,  0, Qt::AlignCenter);

    
    
    QVBoxLayout *vboxManeuverControl = new QVBoxLayout();
    vboxManeuverControl->addLayout(hboxManeuverList);
    vboxManeuverControl->addLayout(vboxManeuverControlButtons);   
    
    QGroupBox *groupBox = new QGroupBox(tr("Maneuver Control"),this);
    groupBox->setLayout(vboxManeuverControl);
    groupBox->setFixedWidth(280);
    
    m_driverManeuverID = new QLabel(this);
    m_driverManeuverID->setText("-");
    m_driverManeuverID->setAlignment(Qt::AlignCenter);
    
    QLabel *labelManeuver = new QLabel(this);
    labelManeuver->setText("Current Maneuver of Driver");
    labelManeuver->setAlignment(Qt::AlignCenter | Qt::AlignCenter);

    m_stateLabel = new QLabel(this);
    m_stateLabel->setFixedSize(140,30);
    m_stateLabel->setAutoFillBackground(true);  
    m_stateLabel->setAlignment(Qt::AlignCenter);  
    setDriverStateError();     
    
    
    m_logField = new QTextEdit(this); 
    m_logField->setReadOnly(true);
    m_logField->setFixedSize(240,180);
        
    QHBoxLayout *hboxStateControl = new QHBoxLayout();    
    hboxStateControl->addWidget(labelManeuver, 0, Qt::AlignCenter);
    hboxStateControl->addWidget(m_driverManeuverID,  0, Qt::AlignCenter); 
    
    QVBoxLayout *vboxStateControl = new QVBoxLayout();
    vboxStateControl->addWidget(m_stateLabel,0, Qt::AlignCenter);
    vboxStateControl->addLayout(hboxStateControl);    
    vboxStateControl->addWidget(m_logField,0, Qt::AlignCenter);
    
    QGroupBox *groupBoxState = new QGroupBox(tr("Driver State"),this);
    groupBoxState->setLayout(vboxStateControl);
    groupBoxState->setFixedWidth(280);
    
    // set the main layout
    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(toplabel,0,Qt::AlignCenter);
    m_mainLayout->addWidget(groupBox, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(groupBoxState, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_pNotAusButton, 0, Qt::AlignCenter);
    setLayout(m_mainLayout);

    //connect the buttons to the slots
    connect(m_pStartButton,  SIGNAL(clicked()), this, SLOT(OnStartButtonClicked()));
    connect(m_pStopButton, SIGNAL(clicked()), this, SLOT(OnStopButtonClicked()));
    connect(m_pRequestButton, SIGNAL(clicked()), this, SLOT(OnStopRequestReadyClicked()));
    connect(m_pNotAusButton, SIGNAL(clicked()), this, SLOT(OnNotAusButtonClicked()));    

    //connect the comboboxes
    connect(m_comboActionBox, SIGNAL(currentIndexChanged(int)), this, SLOT(OnComboActionBoxChanged(int)));
    connect(m_comboSectionBox, SIGNAL(currentIndexChanged(int)), this, SLOT(OnComboSectionBoxChanged(int)));
}


void DisplayWidget::OnStartButtonClicked()
{
    emit sendStart(tInt16(m_comboActionBox->currentText().toInt()));
}

void DisplayWidget::OnStopButtonClicked()
{
    emit sendStop(tInt16(m_comboActionBox->currentText().toInt()));
}

void DisplayWidget::OnStopRequestReadyClicked()
{
    emit sendReadyRequest(tInt16(m_comboActionBox->currentText().toInt()));
}
void DisplayWidget::OnNotAusButtonClicked()
{
    emit sendNotAus();
}

void DisplayWidget::OnDriverState(int state, int entryId)
{
    setDriverManeuverID(entryId);    
    switch (stateCar(state))
    {
    case stateCar_ERROR:
        setDriverStateError();
        break;
    case stateCar_READY:        
        setDriverStateReady();
        break;
    case stateCar_RUNNING:
        setDriverStateRun();
        break;
    case stateCar_COMPLETE:
        setDriverStateComplete();
        break;
    case stateCar_STARTUP:
        break;
    }
}

void DisplayWidget::OnAppendText(QString text)
{
    m_logField->append(text);
}

void DisplayWidget::OnComboActionBoxChanged(int index)
{
    //look for the right section id and write it to section combobox
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        for(unsigned int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
        {
            if(index == m_sectorList[i].maneuverList[j].id)
            {            
                m_comboSectionBox->setCurrentIndex(i);
                break;
            }
        }
    }    
}

void DisplayWidget::OnComboSectionBoxChanged(int index)
{
    //get the action id from selected section and write it to action combo box
    m_comboActionBox->setCurrentIndex(m_sectorList[index].maneuverList[0].id);
}

void DisplayWidget::setDriverStateError()
{
    m_stateLabel->setStyleSheet("background-color: rgb(255,0,0);");
    m_stateLabel->setText("Error");
}
        
void DisplayWidget::setDriverStateRun()
{
    m_stateLabel->setStyleSheet("background-color: rgb(0,255,0);");
    m_stateLabel->setText("Running");
}

void DisplayWidget::setDriverStateReady()
{
    m_stateLabel->setStyleSheet("background-color: rgb(255,255,0);");
    m_stateLabel->setText("Ready");
}

void DisplayWidget::setDriverStateComplete()
{
    m_stateLabel->setStyleSheet("background-color: rgb(0,0,255);");
    m_stateLabel->setText("Complete");
}

void DisplayWidget::setDriverManeuverID(tInt16 id)
{
    m_driverManeuverID->setText(QString::number(id));    
}

void DisplayWidget::SetManeuverList(std::vector<tSector> sectorList)
{
    m_sectorList = sectorList;
}

void DisplayWidget::FillComboBox()
{
    int lastActionId = 0;
    int lastSectionId = 0;

    //get the last action id
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        lastSectionId++;
        for(unsigned int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
            lastActionId++;
    }

    for(int i = 0; i < lastActionId; i++)
    {
        m_comboActionBox->addItem(QString::number(i));
    }

    for(int i = 0; i < lastSectionId; i++)
    {
        m_comboSectionBox->addItem(QString::number(i));
    }
}
