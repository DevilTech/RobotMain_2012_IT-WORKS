package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.microedition.io.FileConnection;
import java.io.OutputStreamWriter;
import javax.microedition.io.Connector;
import java.io.IOException;


//------------------------------------------------------------------------------
public class FileIO {
//------------------------------------------------------------------------------
    FileConnection c;
    OutputStreamWriter writer;

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    FileIO (String filename) throws IOException {
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        String url = "file:///"+filename;

        c = (FileConnection) Connector.open(url,Connector.WRITE);
        c.create();
        writer = new OutputStreamWriter (c.openOutputStream());

    }

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    void write(String data) {
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        try {
            writer.write(data);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    void close() {
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        try {
            c.close();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
