/*************************************************************************
*   Implementation of virtual controller class for SISO system
*   This file is part of labrom_control generic controls package
*   
*   labrom_control is a free sotfware: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_control is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_control.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

#include <string>

//! Top-level namespace
namespace controllers{
//! Abstract controllers class
class Controller{
  public:
    //! Empty Constructor [AVOID]
    Controller(){};
    //! Construct that receives name identifier [PREFERRED]
    Controller(std::string name_id): name_id_(name_id){};
    //! Destructor
    ~Controller(void){};
    //! Loop once
    virtual double LoopOnce(double ref, double feedback, double dt, double d_ref=0)=0;   
    //! Get output value
    virtual double GetOutput(void) = 0;

  protected:
    std::string name_id_;             //!< Identification
    std::string name_type_   ;             //!< Controller type e.g. pid, mpc, lqr..         
};

} // namespace controllers
#endif // CONTROLLERS_H_
