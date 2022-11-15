#include "ModelDialog.h"
#include "ui_ModelDialog.h"

//#include "../../ALPrograms/HuboQuad/joint_inverse.h"
#include "../../ALPrograms/Quadruped/joint_inverse_SW.h"
//#include "../../ALPrograms/HuboQuad/Oinverse.h"
using namespace isnl;

struct QuadParams
{
    double P2HX,P2HY,ULEG,LLEG,R2P;
    QuadParams()
    {
//        P2HX = 0.45*0.5;
//        P2HY = 0.23*0.5;
//        ULEG = 0.25;
//        LLEG = 0.22;
//        R2P = 0.078;
        P2HX = 0.231;
        P2HY = 0.110;
        ULEG = 0.28;
        LLEG = 0.28;
        R2P = 0.047;
    }
};



enum JOINT_ID{
    J_LSR,J_LSP,J_LEB,
    J_RSR,J_RSP,J_REB,
    J_LHR,J_LHP,J_LKN,
    J_RHR,J_RHP,J_RKN,
    NJOINT
};


enum BONE_ID{
    B_WST,
    B_LSR,B_LSP,B_LEB,
    B_RSR,B_RSP,B_REB,
    B_LHR,B_LHP,B_LKN,
    B_RHR,B_RHP,B_RKN,
    NBONE
};
enum COMP_ID{
    JLSR,JLSP,JLEB,
    JRSR,JRSP,JREB,
    JLHR,JLHP,JLKN,
    JRHR,JRHP,JRKN,
    BWST,
    BLSR,BLSP,BLEB,
    BRSR,BRSP,BREB,
    BLHR,BLHP,BLKN,
    BRHR,BRHP,BRKN,
    NCOMP
};
static const char COMP_NAME[NCOMP+1][10] = {
    "JLSR","JLSP","JLEB",
    "JRSR","JRSP","JREB",
    "JLHR","JLHP","JLKN",
    "JRHR","JRHP","JRKN",
    "BWST",
    "BLSR","BLSP","BLEB",
    "BRSR","BRSP","BREB",
    "BLHR","BLHP","BLKN",
    "BRHR","BRHP","BRKN",
    "null"
};


std::string STLPath = "../share/GUI/stl/";

ModelDialog::ModelDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelDialog)
{
    ui->setupUi(this);

    glWidget = new GLWidget(this);

    QScrollArea *glWidgetArea = new QScrollArea;
    glWidgetArea->setWidget(glWidget);
    glWidgetArea->setWidgetResizable(true);
    glWidgetArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    glWidgetArea->setMinimumSize(50, 50);

    xSlider = createSlider(SIGNAL(xRotationChanged(int)), SLOT(setXRotation(int)));
    ySlider = createSlider(SIGNAL(yRotationChanged(int)), SLOT(setYRotation(int)));
    zSlider = createSlider(SIGNAL(zRotationChanged(int)), SLOT(setZRotation(int)));

    ui->LAYOUT_MODEL->addWidget(glWidgetArea, 0, 0);
    ui->LAYOUT_SLIDER->addWidget(xSlider,0,0);
    ui->LAYOUT_SLIDER->addWidget(ySlider,1,0);
    ui->LAYOUT_SLIDER->addWidget(zSlider,2,0);

    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(180 * 16);

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(30);
    //connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(DisplayUpdate()));
}

ModelDialog::~ModelDialog()
{
    delete ui;
}


void ModelDialog::DisplayUpdate(){
    ONELEGFKIK Oii;
    if(ui->CB_USE_ENCODER->isChecked()){
//        glWidget->model->ref[J_LSR+7] = JointEncoder(LSR) * D2Rf;
//        glWidget->model->ref[J_LSP+7] = Oii.m2q_lhp(JointEncoder(LSP));
//        glWidget->model->ref[J_LEB+7] = Oii.m2q_kn(JointEncoder(LEB));

//        glWidget->model->ref[J_RSR+7] = JointEncoder(RSR) * D2Rf;
//        glWidget->model->ref[J_RSP+7] = Oii.m2q_rhp(JointEncoder(RSP));
//        glWidget->model->ref[J_REB+7] = Oii.m2q_kn(JointEncoder(REB));


//        glWidget->model->ref[J_LHR+7] = JointEncoder(LHR) * D2Rf;
//        glWidget->model->ref[J_LHP+7] = Oii.m2q_lhp(JointEncoder(LHP));
//        glWidget->model->ref[J_LKN+7] = Oii.m2q_kn(JointEncoder(LKN));

//        glWidget->model->ref[J_RHR+7] = JointEncoder(RHR) * D2Rf;
//        glWidget->model->ref[J_RHP+7] = Oii.m2q_rhp(JointEncoder(RHP));
//        glWidget->model->ref[J_RKN+7] = Oii.m2q_kn(JointEncoder(RKN));

//        glWidget->model->ref[J_LSR+7] = Oii.m2q_lhr(JointEncoder(LSR)*D2Rf);
//        glWidget->model->ref[J_LSP+7] = JointEncoder(LSP)*D2Rf;
//        glWidget->model->ref[J_LEB+7] = Oii.m2q_kn(JointEncoder(LEB)*D2Rf);

//        glWidget->model->ref[J_RSR+7] = Oii.m2q_rhr(JointEncoder(RSR)*D2Rf);
//        glWidget->model->ref[J_RSP+7] = JointEncoder(RSP)*D2Rf;
//        glWidget->model->ref[J_REB+7] = Oii.m2q_kn(JointEncoder(REB)*D2Rf);


        glWidget->model->ref[J_LHR+7] = Oii.m2q_lhr(JointEncoder(LHR)*D2Rf);
        glWidget->model->ref[J_LHP+7] = JointEncoder(LHP)*D2Rf;
        glWidget->model->ref[J_LKN+7] = Oii.m2q_kn(JointEncoder(LKN)*D2Rf);

        glWidget->model->ref[J_RHR+7] = Oii.m2q_rhr(JointEncoder(RHR)*D2Rf);
        glWidget->model->ref[J_RHP+7] = JointEncoder(RHP)*D2Rf;
        glWidget->model->ref[J_RKN+7] = Oii.m2q_kn(JointEncoder(RKN)*D2Rf);

    }else{
//        glWidget->model->ref[J_LSR+7] = JointReference(LSR) * D2Rf;
//        glWidget->model->ref[J_LSP+7] = Oii.m2q_lhp(JointReference(LSP));
//        glWidget->model->ref[J_LEB+7] = Oii.m2q_kn(JointReference(LEB));

//        glWidget->model->ref[J_RSR+7] = JointReference(RSR) * D2Rf;
//        glWidget->model->ref[J_RSP+7] = Oii.m2q_rhp(JointReference(RSP));
//        glWidget->model->ref[J_REB+7] = Oii.m2q_kn(JointReference(REB));

//        glWidget->model->ref[J_LHR+7] = JointReference(LHR) * D2Rf;
//        glWidget->model->ref[J_LHP+7] = Oii.m2q_lhp(JointReference(LHP));
//        glWidget->model->ref[J_LKN+7] = Oii.m2q_kn(JointReference(LKN));

//        glWidget->model->ref[J_RHR+7] = JointReference(RHR) * D2Rf;
//        glWidget->model->ref[J_RHP+7] = Oii.m2q_rhp(JointReference(RHP));
//        glWidget->model->ref[J_RKN+7] = Oii.m2q_kn(JointReference(RKN));

//        glWidget->model->ref[J_LSR+7] = Oii.m2q_lhr(JointReference(LSR)*D2Rf);
//        glWidget->model->ref[J_LSP+7] = JointReference(LSP)*D2Rf;
//        glWidget->model->ref[J_LEB+7] = Oii.m2q_kn(JointReference(LEB)*D2Rf);

//        glWidget->model->ref[J_RSR+7] = Oii.m2q_rhr(JointReference(RSR)*D2Rf);
//        glWidget->model->ref[J_RSP+7] = JointReference(RSP)*D2Rf;
//        glWidget->model->ref[J_REB+7] = Oii.m2q_kn(JointReference(REB)*D2Rf);

        glWidget->model->ref[J_LHR+7] = Oii.m2q_lhr(JointReference(LHR)*D2Rf);
        glWidget->model->ref[J_LHP+7] = JointReference(LHP)*D2Rf;
        glWidget->model->ref[J_LKN+7] = Oii.m2q_kn(JointReference(LKN)*D2Rf);

        glWidget->model->ref[J_RHR+7] = Oii.m2q_rhr(JointReference(RHR)*D2Rf);
        glWidget->model->ref[J_RHP+7] = JointReference(RHP)*D2Rf;
        glWidget->model->ref[J_RKN+7] = Oii.m2q_kn(JointReference(RKN)*D2Rf);

    }
    glWidget->model->ref[3] = PODO_DATA.UserM2G.qPel[0];
    glWidget->model->ref[4] = PODO_DATA.UserM2G.qPel[1];
    glWidget->model->ref[5] = PODO_DATA.UserM2G.qPel[2];
    glWidget->model->ref[6] = PODO_DATA.UserM2G.qPel[3];



    glWidget->updateGL();
}


QSlider* ModelDialog::createSlider(const char *changedSignal, const char *setterSlot){
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    connect(slider, SIGNAL(valueChanged(int)), glWidget, setterSlot);
    connect(glWidget, changedSignal, slider, SLOT(setValue(int)));
    return slider;
}

void ModelDialog::on_BT_CamFront_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(180 * 16);
}
void ModelDialog::on_BT_CamRight_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(270 * 16);
}
void ModelDialog::on_BT_CamLeft_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue( 90 * 16);
}


GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    xRot = 0;
    yRot = 0;
    zRot = 0;



    model = newModel();
    model->ref.resize(NJOINT+7);//NO_OF_JOINTS+7);
    model->ref[3] = 1.f;

    //globjs << model;




}
GLWidget::~GLWidget()
{
    makeCurrent();
}

void GLWidget::setXRotation(int angle){
    normalizeAngle(&angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
    }
}
void GLWidget::setYRotation(int angle){
    normalizeAngle(&angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
    }
}
void GLWidget::setZRotation(int angle){
    normalizeAngle(&angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
    }
}
void GLWidget::initializeGL()
{
    GLfloat lightColor[] = {1.0f, 1.0f, 1.0f, 1.0f};

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor);
    glEnable(GL_DEPTH_TEST);

    globjs.initialize();

    glEnable(GL_NORMALIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}
void GLWidget::paintGL()
{
    GLfloat lightPos0[4] = { 10.0f,-10.0f, 7.0f, 1.0f };
    GLfloat lightPos1[4] = {-10.0f, 10.0f, 7.0f, 1.0f };
    GLfloat lightPos2[4] = {-10.0f,-10.0f, 7.0f, 1.0f };

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glPushMatrix();

    glRotated(xRot / 16.0 -180, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0 -180, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0 -180, 0.0, 0.0, 1.0);

    model->render();



    glPushMatrix();

    glRotated(xRot / 16.0 -180, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0 -180, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0 -180, 0.0, 0.0, 1.0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
    glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);


    globjs.render();

    glPopMatrix();

    glPopMatrix();

}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-0.1, +0.1, -0.1, 0.1, 0.1, 60.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -1.0);
    // change camera coord to world coord :
    glRotatef(2.0944*R2Df, -0.5774, -0.5774, -0.5774);
}
void GLWidget::mousePressEvent(QMouseEvent *event){
    lastPos = event->pos();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event){
    currPos = event->pos();
    int dx = currPos.x() - lastPos.x();
    int dy = currPos.y() - lastPos.y();

    if(event->buttons() & Qt::RightButton){
        setYRotation(yRot + 2 * dy);
        setZRotation(zRot + 2 * dx);
    }else if (event->buttons() & Qt::LeftButton){
//        setXRotation(xRot + 8 * dy);
//        setZRotation(zRot + 8 * dx);
        glTranslatef(0, dx/500.0, -dy/500.0);
    }
    lastPos = event->pos();
}
void GLWidget::wheelEvent(QWheelEvent *event){
    //if(event->modifiers().testFlag(Qt::ControlModifier)){
        glTranslatef(-event->delta()/1000.0, 0, 0);
    //}
}

void GLWidget::normalizeAngle(int *angle){
    while (*angle < 0)
        *angle += 360 * 16;
    while (*angle > 360 * 16)
        *angle -= 360 * 16;
}


// ===========================
// ===========================


HUBOModel* GLWidget::newModel(){

    Joints joints(NJOINT);
    Bones  bones(NBONE);
    Bones tempbone(35);
    std::vector<GLComplex*> objs(1);
    objs[0] = new GLComplex();

//    // Lower body joints
//    joints[J_LHR]     = new RevoluteXJoint("LSR");
//    joints[J_LHP]     = new RevoluteYJoint("LSP");
//    joints[J_LKN]     = new RevoluteYJoint("LKN");
//    joints[J_RHR]     = new RevoluteXJoint("RHR");
//    joints[J_RHP]     = new RevoluteYJoint("RHP");
//    joints[J_RKN]     = new RevoluteYJoint("RKN");
//    // Upper body joints
//    joints[J_LSR]     = new RevoluteXJoint("LSR");
//    joints[J_LSP]     = new RevoluteYJoint("LSP");
//    joints[J_LEB]     = new RevoluteYJoint("LEB");
//    joints[J_RSR]     = new RevoluteXJoint("RSR");
//    joints[J_RSP]     = new RevoluteYJoint("RSP");
//    joints[J_REB]     = new RevoluteYJoint("REB");


//    GLObject *body_wst	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_WST_col.STL");

//    //LARM
//    GLObject *body_lsp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LSP_col.STL");
//    GLObject *body_lsr     = newStl(0.0, 0.0, 0.0, STLPath+"Body_LSR_col.STL");
//    GLObject *body_leb     = newStl(0.0, 0.0, 0.0, STLPath+"Body_LEB_col.STL");

//    //RARM
//    GLObject *body_rsp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RSP_col.STL");
//    GLObject *body_rsr     = newStl(0.0, 0.0, 0.0, STLPath+"Body_RSR_col.STL");
//    GLObject *body_reb     = newStl(0.0, 0.0, 0.0, STLPath+"Body_REB_col.STL");
//    //LLEG
//    GLObject *body_lhr	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LHR_col.STL");
//    GLObject *body_lhp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LHP_col.STL");
//    GLObject *body_lkn	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LKN_col.STL");

//    //RLEG
//    GLObject *body_rhr	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RHR_col.STL");
//    GLObject *body_rhp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RHP_col.STL");
//    GLObject *body_rkn	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RKN_col.STL");


    QuadParams Oi;
    GLObject *body_wst	   = newBox(0.0,0.0,-0.03,Oi.P2HX*2+0.2,Oi.P2HY*2,0.15);

    //LARM
    GLObject *body_lsr	   = newBox(-0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_lsp     = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_leb     = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);

    //RARM
    GLObject *body_rsr	   = newBox(-0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_rsp     = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_reb     = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);
    //LLEG
    GLObject *body_lhr	   = newBox(+0.05,0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_lhp	   = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_lkn	   = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);

    //RLEG
    GLObject *body_rhr	   = newBox(+0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_rhp	   = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_rkn	   = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);


//    bones[B_WST]    = new Bone(COMP_NAME[BWST],    isnl::pos(0.0, 0.0, 0.0), body_wst);

//    bones[B_LSP]    = new Bone(COMP_NAME[BLSP],    isnl::pos(0.0, 0.0, 0.0), body_lsp);
//    bones[B_LSR]    = new Bone(COMP_NAME[BLSR],    isnl::pos(0.0, 0.0, 0.0), body_lsr);
//    bones[B_LEB]    = new Bone(COMP_NAME[BLEB],    isnl::pos(0.0, 0.0, 0.0), body_leb);

//    bones[B_RSP]    = new Bone(COMP_NAME[BRSP],    isnl::pos(0.0, 0.0, 0.0), body_rsp);
//    bones[B_RSR]    = new Bone(COMP_NAME[BRSR],    isnl::pos(0.0, 0.0, 0.0), body_rsr);
//    bones[B_REB]    = new Bone(COMP_NAME[BREB],    isnl::pos(0.0, 0.0, 0.0), body_reb);

//    bones[B_LHR]    = new Bone(COMP_NAME[BLHR],    isnl::pos(0.0, 0.0, 0.0), body_lhr);
//    bones[B_LHP]    = new Bone(COMP_NAME[BLHP],    isnl::pos(0.0, 0.0, 0.0), body_lhp);
//    bones[B_LKN]    = new Bone(COMP_NAME[BLKN],    isnl::pos(0.0, 0.0, 0.0), body_lkn);

//    bones[B_RHR]    = new Bone(COMP_NAME[BRHR],    isnl::pos(0.0, 0.0, 0.0), body_rhr);
//    bones[B_RHP]    = new Bone(COMP_NAME[BRHP],    isnl::pos(0.0, 0.0, 0.0), body_rhp);
//    bones[B_RKN]    = new Bone(COMP_NAME[BRKN],    isnl::pos(0.0, 0.0, 0.0), body_rkn);


//    tempbone[0]  = new Bone("T_LS",       isnl::pos(Oi.P2HX,Oi.P2HY,0.0));
//    tempbone[1]  = new Bone("LS_LS2",       isnl::pos(0.0,Oi.R2P,0.0));
//    tempbone[2]  = new Bone("LS2_LEB",     isnl::pos(0.0,0.0,-Oi.ULEG));
//    tempbone[3]  = new Bone("T_RS",       isnl::pos(Oi.P2HX,-Oi.P2HY,0.0));
//    tempbone[4]  = new Bone("RS_RS2",       isnl::pos(0.0,-Oi.R2P,0.0));
//    tempbone[5]  = new Bone("RS2_REB",    isnl::pos(0.0,0.0,-Oi.ULEG));

//    tempbone[6]  = new Bone("T_LH",        isnl::pos(-Oi.P2HX,Oi.P2HY,0.0));
//    tempbone[7]  = new Bone("LH_LH2",        isnl::pos(0.0,Oi.R2P,0.0));
//    tempbone[8]  = new Bone("LH2_LKN",      isnl::pos(0.0,0.0,-Oi.ULEG));
//    tempbone[9]  = new Bone("T_RH",        isnl::pos(-Oi.P2HX,-Oi.P2HY,0.0));
//    tempbone[10]  = new Bone("RH_RH2",       isnl::pos(0.0,-Oi.R2P,0.0));
//    tempbone[11]  = new Bone("RH2_RKN",    isnl::pos(0.0,0.0,-Oi.ULEG));

    //tree
    bones[B_WST]->setParent(NULL);
    *bones[B_WST] + tempbone[0]+joints[J_LSR] + bones[B_LSR] + tempbone[1]+ joints[J_LSP] + bones[B_LSP]
            + tempbone[2]+ joints[J_LEB] + bones[B_LEB];

    *bones[B_WST] + tempbone[3]+joints[J_RSR] + bones[B_RSR] + tempbone[4]+ joints[J_RSP] + bones[B_RSP]
            + tempbone[5]+ joints[J_REB] + bones[B_REB];


    *bones[B_WST] + tempbone[6]+joints[J_LHR] + bones[B_LHR] + tempbone[7]+ joints[J_LHP] + bones[B_LHP]
            + tempbone[8]+ joints[J_LKN] + bones[B_LKN];

    *bones[B_WST] + tempbone[9]+joints[J_RHR] + bones[B_RHR] + tempbone[10]+ joints[J_RHP] + bones[B_RHP]
            + tempbone[11]+ joints[J_RKN] + bones[B_RKN];





    return new HUBOModel(bones, joints, objs);
}
