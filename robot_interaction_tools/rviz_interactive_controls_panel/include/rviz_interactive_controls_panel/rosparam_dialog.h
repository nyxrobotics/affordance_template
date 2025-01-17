/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef ROSPARAM_DIALOG_HPP
#define ROSPARAM_DIALOG_HPP

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <QDialog>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <unordered_set>
#include <QSpinBox>


class DoubleEditor : public QDoubleSpinBox
{
  Q_OBJECT
  
 public:
  DoubleEditor(QWidget *widget = 0) : QDoubleSpinBox(widget) {
    this->setDecimals(4);
  }
  
};


namespace rviz_interactive_controls_panel {
	
	class RosparamTreeWidget;
	class RosparamTreeItem;
	
	/** The \c QDialog that contains a \c QTreeWidget of ROS parameters. */
	class RosparamDialog : public QDialog {
		Q_OBJECT
		public:
			RosparamDialog(const std::string &ns, QWidget *parent = 0);
			~RosparamDialog();
		private Q_SLOTS:
			/** Process a click of the \a OK button (accepts any changes). */
			void okClicked();
			/** Process a click of the \a Cancel button (discards changes). */
			void noClicked();
			/** Receive notice that the enclosed \c QTreeWidget has determined
			 * the size required to display its columns. */
			void onTreeWidthCalculated(int width);
		private:
			/** Create the \c RosparamTreeWidget, using \c ns as the root. */
			void createRosparamTreeWidget(const std::string &ns);
			/** Resize, accounting for margins, spacing, etc. */
			void resizeDialog();
			/** Executed prior to closing; updates any changed parameter
			 * values. */
			void processClick(QDialog::DialogCode code);
			/** Lays out the widgets; assumes all widgets exist. */
			void setupUi();
			
			XmlRpc::XmlRpcValue m_root;
			RosparamTreeWidget *m_treeWidget;
			QPushButton *m_ok, *m_no;
			int m_treeDispWidth;
	};
	
	/** A \c QTreeWidget representing (editable) ROS parameters. */
	class RosparamTreeWidget : public QTreeWidget {
		Q_OBJECT
		public:
			RosparamTreeWidget(RosparamDialog* parent);
			/** Fill the parameter tree. */
			void fillTree(XmlRpc::XmlRpcValue* rootVal,
			              const std::string &rootPath);
			/** Set any changed values on the ROS parameter server. */
			void commitUpdates(ros::NodeHandle &nh);
			/** A \a user_type for Qt. */
			static int rosparamItemType;
		Q_SIGNALS:
			/** A signal containing the widget width required to display the
			 * fully expanded tree view contents. */
			void treeWidthCalculated(int);
		private Q_SLOTS:
			/** Handle item editing (on double click). */
			void onItemDoubleClicked(QTreeWidgetItem* item, int col);
			/** Record that items changed for update on exit. */
			void onItemChanged(QTreeWidgetItem* item, int col);
		private:
			/** Function to limit which columns' values are editable. */
			bool isEditable(int col);
			
			RosparamTreeItem* m_rootItem;
			std::unordered_set<RosparamTreeItem*> m_updates;
	};
	
	/** Items that appear as entries in a \c RosparamTreeWidget. A
	 * \c RosparamTreeItem is responsible for translating between \c XmlRpc
	 * and \c QVariant values, maintaining the parameter's path and name,
	 * creating its children parameters (recursively), and setting values
	 * on the parameter server.
	 * 
	 * At this point, editing relies on the default \c QItemEditorFactory
	 * for editing widgets (e.g., \c QComboBox for \c bool, \c QLineEdit
	 * for \c QString, etc.); in the future, it is desirable to provide
	 * facilities for custom editing, particularly a \c QComboBox for
	 * enumerations. Note that, according to the \c QStyledItemDelegate
	 * documentation, it is possible to provide editors without using an
	 * editor factory (see \c QStyledItemDelegate::createEditor). */
	class RosparamTreeItem : public QTreeWidgetItem {
		public:
			/** Constructor. */
			RosparamTreeItem(XmlRpc::XmlRpcValue* val,
			                 RosparamTreeItem* parent,
			                 const std::string &path);
			/** Return the \c XmlRpcValue as a \c QVariant. */
			QVariant data();
			/** Set the \a current value on the ROS parameter server. Note
			 * that the current value is stored in this \c QTreeWidgetItem's
			 * editable column as a \c QVariant. The value of the initial
			 * \c XmlRpcValue is used to detect changes. */
			bool setParam(const ros::NodeHandle &nh);
		private:
			/** Recursively create children (when receiving an
			 * \c XmlRpc::XmlRpcValue::TypeStruct). */
			void createChildren();
			void addChild(const std::string &name,
			              XmlRpc::XmlRpcValue* xmlVal);
			/** Translate an \c XmlRpcValue to a \c QVariant. */
			QVariant valToQVariant(XmlRpc::XmlRpcValue &val) const;
			
			XmlRpc::XmlRpcValue* m_val;
			std::string m_nsPath;
	};
	
}
#endif

