final int WIDTH = 700;
final int HEIGHT = 700;

int x1= int(WIDTH / 7);
int y1= int(WIDTH / 7);
int len=int(WIDTH / 7);
int rowh = 6;
int colh=5;

int[][] Horizontalmap = {

  { 1, 0, 1, 1, 1},
  { 0, 1, 1, 1, 0},
  { 1, 0, 0, 1, 0 },
  { 0, 1, 1, 0, 0 },
  { 0, 0, 0, 0, 0 },
  { 1, 1, 0, 1, 1 },

};

int[][] Verticalmap = {

  { 1, 0, 0, 0, 0, 1},
  { 1, 0, 1, 0, 0, 1},
  { 1, 0, 1, 1, 1, 1 },
  { 1, 1, 0, 1, 1, 1 },
  { 1, 0, 1, 0, 1, 1 },

}; 
 
 
void setup() {
  size(700, 700);
  noLoop(); 

}
 
void draw() {

 
  for (int j = 0; j <rowh ; j ++) {
    for(int i = 0; i < colh; i ++)  {
      if ( Horizontalmap[j][i] == 1) {
        strokeWeight(8);
        line(x1+i*len,y1+j*len,x1+(i+1)*len,y1+j*len) ; 
        stroke(0);
      }    
    } 
  }
    for (int j = 0; j <colh ; j ++) {
    for(int i = 0; i < rowh; i ++)  {
      if ( Verticalmap[j][i] == 1) {
        strokeWeight(8);
        line(x1+i*len,y1+j*len,x1+i*len,y1+(j+1)*len) ; 
      }
    } 
  } 


 
}
