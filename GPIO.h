#ifndef GPIO_CLASS_H
#define GPIO_CLASS_H
 
#include <string>
using namespace std;
/** GPIO Class
 * Purpose: Each object instantiated from this class will control a GPIO pin
 * The GPIO pin number must be passed to the overloaded class constructor
 */
class GPIO
{
public:
    
    /** Creates a GPIO object */
    GPIO();  // create a GPIO object that controls GPIO4 (default
    
    /** Creates a GPIO object for the specified pin
     * @param x the GPIO pin to control
     */
    GPIO(string x);
    
    /** Sets up a GPIO pin
     * @return error code
     */
    int export_gpio();
    
    /** Removes control of the GPIO pin
     * @return error code
     */
    int unexport_gpio();
    
    /** Sets if the GPIO pin is input or output
     * @param dir input("in") or output("out")
     * @return error code
     */
    int setdir_gpio(string dir);
    
    /** Sets the state of the GPIO pin
     * @param val the state of the pin ("1" or "0")
     * @return error code
     */
    int setval_gpio(string val);
    
    /** Reads the state of the GPIO pin
     * @param val a reference to a variable that will hold the state of the GPIO pin
     * @return error code
     */
    int getval_gpio(string& val);
    
    /** Gets the GPIO pin number for this object
     * @return the GPIO pin number for this object
     */
    string get_gpionum();
private:
    string gpionum; // GPIO number associated with the instance of an object
};
 
#endif