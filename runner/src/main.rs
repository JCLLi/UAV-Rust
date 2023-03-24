mod interface;
use crate::interface::interface::setup_interface;

fn main()  {
    // Clear terminal
    print! ("\x1B[2J\x1B[1;1H");

    let res = setup_interface();
    
    print!("\x1B[2J\x1B[1;1H");
    println!("\rInterface stopped: {:?}", res);
}