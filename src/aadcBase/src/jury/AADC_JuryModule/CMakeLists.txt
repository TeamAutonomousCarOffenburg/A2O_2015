set(FILTER_NAME aadc_juryModule)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found 
if(QT_FOUND)

    set(HEADER_MOC
        cJuryModule.h
        displaywidget.h
        )
        
    adtf_add_moc(${FILTER_NAME}
                    ${HEADER_MOC}
                    )
    
    adtf_add_filter(${FILTER_NAME}
                    stdafx.cpp
                    stdafx.h	
                    displaywidget.cpp
                    cJuryModule.cpp
                    ${HEADER_MOC}
    )
    
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} aadcbase/jury) 
    
    adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
    
endif(QT_FOUND)



