/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package tempixTerminalPackage;

import java.util.Timer;
import java.util.TimerTask;


/**
 *
 * @author Brigitte
 */
public class TempixTerminal extends javax.swing.JFrame {
    private JSerialCommsController commsController;
    private static TempixTerminal tempixTerm;
    
    private static int cnt = 0;

    /**
     * Creates new form tempixTerminalUI
     */
    public TempixTerminal() {
        commsController = new JSerialCommsController();
        initComponents();
    }
    
    public void listPortsNOpenFirstNOnly()
    {
        try {
            String res = commsController.listPortsNOpenFirstNOnly();
            addToLog(res);
        } catch(Throwable ex)  {
            addToLog("exception in openCommsController: "+ ex.getMessage());
        }
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        UartCommButton = new javax.swing.JButton();
        jLogTextScrollPane = new javax.swing.JScrollPane();
        jLogTextArea = new javax.swing.JTextArea();
        jLabel1 = new javax.swing.JLabel();
        clearButton = new javax.swing.JButton();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setTitle("TempixTerminal");
        addWindowListener(new java.awt.event.WindowAdapter() {
            public void windowOpened(java.awt.event.WindowEvent evt) {
                formWindowOpened(evt);
            }
        });

        UartCommButton.setText("UartComm");
        UartCommButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                UartCommButtonActionPerformed(evt);
            }
        });

        jLogTextScrollPane.setFocusable(false);

        jLogTextArea.setEditable(false);
        jLogTextArea.setColumns(20);
        jLogTextArea.setRows(5);
        jLogTextScrollPane.setViewportView(jLogTextArea);

        jLabel1.setText("log");

        clearButton.setText("clear");
        clearButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                clearButtonActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLogTextScrollPane, javax.swing.GroupLayout.DEFAULT_SIZE, 644, Short.MAX_VALUE)
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(jLabel1)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(clearButton))
                            .addComponent(UartCommButton))
                        .addGap(0, 0, Short.MAX_VALUE)))
                .addGap(19, 19, 19))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(UartCommButton)
                .addGap(11, 11, 11)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(jLabel1)
                    .addComponent(clearButton))
                .addGap(4, 4, 4)
                .addComponent(jLogTextScrollPane, javax.swing.GroupLayout.DEFAULT_SIZE, 445, Short.MAX_VALUE)
                .addGap(21, 21, 21))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void UartCommButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_UartCommButtonActionPerformed
        // TODO add your handling code here:
        if (evt.getSource().equals( UartCommButton)) {
            CommDialog.createAndShowGUI(this);
        } 
        
    }//GEN-LAST:event_UartCommButtonActionPerformed

    private void formWindowOpened(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_formWindowOpened
        // TODO add your handling code here:
        listPortsNOpenFirstNOnly();
    }//GEN-LAST:event_formWindowOpened

    private void clearButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_clearButtonActionPerformed
        // TODO add your handling code here:
        jLogTextArea.setText("");
    }//GEN-LAST:event_clearButtonActionPerformed

    public void addToLog(String txt)
    {
       java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                jLogTextArea.append(txt);
                jLogTextArea.setCaretPosition(jLogTextArea.getText().length());
            }
        }); 
    }
    
    public static void addToLogger(String str)
    {
        if (tempixTerm != null) {  
            tempixTerm.addToLog(str);
        }
    }
    
    public static void availableDataReceived(String str)
    {
        if (tempixTerm != null) {  
            tempixTerm.addToLog(str);
        }
        
    }
    
  
    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(TempixTerminal.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(TempixTerminal.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(TempixTerminal.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(TempixTerminal.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>
        //</editor-fold>
        //</editor-fold>
        //</editor-fold>
        

 

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                tempixTerm= new TempixTerminal();
                tempixTerm.setVisible(true);
            }
        });
        
        
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton UartCommButton;
    private javax.swing.JButton clearButton;
    private javax.swing.JLabel jLabel1;
    private javax.swing.JTextArea jLogTextArea;
    private javax.swing.JScrollPane jLogTextScrollPane;
    // End of variables declaration//GEN-END:variables
}
