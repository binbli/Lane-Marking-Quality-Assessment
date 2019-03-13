#ifndef BaseLD_H_
#define BaseLD_H_

#include<iostream>
#include<vector>
#include<string>
#include<algorithm>
#include<math.h>
#include<pugixml.hpp>
#include<exception>

namespace LD {

    using std::cout;
	using std::endl;
	using std::string;
	using std::vector;
	using std::runtime_error;

	class BaseLD {

    protected:
        string m_xmlFileName;
        pugi::xml_node m_xml;
        pugi::xml_document m_xmlDoc;

        virtual void ParseXML() = 0;

    public:

        typedef unsigned long long int ulli;

        bool m_debug; //to be changed

        virtual void ParseXML(string _file) final {
            pugi::xml_parse_result docStatus = m_xmlDoc.load_file(_file.c_str());

            if (!docStatus)
                throw runtime_error("Error with " + _file + ": " + docStatus.description());

            m_xml = m_xmlDoc.document_element();

            m_debug = m_xml.child("Main").attribute("debug").as_bool();
        }

        BaseLD(string _file) : m_xmlFileName(_file) {
            ParseXML(_file); //will call above function
        }

	};

}

#endif
